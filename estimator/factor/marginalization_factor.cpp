#include "marginalization_factor.h"
#include "utility/print.h"

void ResidualBlockInfo::Evaluate()
{
	residuals.resize(cost_function->num_residuals());

	const auto& block_sizes = cost_function->parameter_block_sizes();
	raw_jacobians = new double *[block_sizes.size()];
	jacobians.resize(block_sizes.size());

	for (int i = 0; i < static_cast<int>(block_sizes.size()); ++i) {
		jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
		raw_jacobians[i] = jacobians[i].data();
	}
	cost_function->Evaluate(parameter_blocks.data(), residuals.data(), raw_jacobians);

    //std::vector<int> tmp_idx(block_sizes.size());
    //Eigen::MatrixXd tmp(dim, dim);
    //for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
    //{
    //    int size_i = localSize(block_sizes[i]);
    //    Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
    //    for (int j = 0, sub_idx = 0; j < static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] == 7 ? 6 : block_sizes[j], j++)
    //    {
    //        int size_j = localSize(block_sizes[j]);
    //        Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
    //        tmp_idx[j] = sub_idx;
    //        tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) = jacobian_i.transpose() * jacobian_j;
    //    }
    //}
    //Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
    //std::cout << saes.eigenvalues() << std::endl;
    //ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

	// 根据损失函数类型求解调整后的残差和雅可比
    if (loss_function) {
		double residual_scaling_, alpha_sq_norm_;
		double sq_norm, rho[3];

        sq_norm = residuals.squaredNorm();
		loss_function->Evaluate(sq_norm, rho);

		double sqrt_rho1_ = sqrt(rho[1]);

        if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
			residual_scaling_ = sqrt_rho1_;
			alpha_sq_norm_ = 0.0;
        }
        else {
            const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
            const double alpha = 1.0 - sqrt(D);
            residual_scaling_ = sqrt_rho1_ / (1 - alpha);
            alpha_sq_norm_ = alpha / sq_norm;
        }

        for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
            jacobians[i] = sqrt_rho1_ * (jacobians[i] - alpha_sq_norm_ * residuals * (residuals.transpose() * jacobians[i]));
        }

		residuals *= residual_scaling_;
    }
}

MarginalizationInfo::~MarginalizationInfo()
{
	console::print_info("INFO: release marginlization info.\n");
	for (auto it = parameter_block_data.begin(); it != parameter_block_data.end(); ++it)
		delete it->second;

    for (int i = 0; i < (int)factors.size(); ++i) {
        delete[] factors[i]->raw_jacobians;
		delete factors[i]->cost_function;
        delete factors[i];
    }
}

void MarginalizationInfo::addResidualBlockInfo(ResidualBlockInfo *residual_block_info)
{
	factors.emplace_back(residual_block_info);

	const auto& parameter_blocks = residual_block_info->parameter_blocks;
	const auto& parameter_block_sizes = residual_block_info->cost_function->parameter_block_sizes();

	double *addr = nullptr;

	// 将所有优化变量的size信息记录下来
    for (int i = 0; i < static_cast<int>(residual_block_info->parameter_blocks.size()); ++i) {
		addr = parameter_blocks[i];
		parameter_block_size[reinterpret_cast<long>(addr)] = parameter_block_sizes[i];
    }

	// 把需要边缘化的优化变量放在前面记录下来
    for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size()); ++i) {
		addr = parameter_blocks[residual_block_info->drop_set[i]];
		parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
    }
}

void MarginalizationInfo::preMarginalize()
{
	// 将factor中所有优化变量添加到成员变量parameter_block_data中
    for (const auto& it : factors) {
		it->Evaluate();
		const auto& block_sizes = it->cost_function->parameter_block_sizes();
        for (int i = 0; i < static_cast<int>(block_sizes.size()); ++i) {
			long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
			int size = block_sizes[i];
            if (parameter_block_data.find(addr) == parameter_block_data.end()) {
				double *data = new double[size];
				memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
				parameter_block_data[addr] = data;
            }
        }
    }
}

int MarginalizationInfo::localSize(int size) const
{
	return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const
{
	return size == 6 ? 7 : size;
}

void* ThreadsConstructA2(void* threadsstruct, 
	 std::unordered_map<long, int>& parameter_block_size,
	 std::unordered_map<long, int>& parameter_block_idx)
{
	ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
	for (const auto& it : p->sub_factors) {
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); ++i) {
			int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
			int size_i = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
			if (size_i == 7) size_i = 6;
			Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
			for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); ++j) {
				int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
				int size_j = parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
				if (size_j == 7) size_j = 6;
				Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
				if (i == j) {
					p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
				}
				else {
					//std::cout << "p->A: " << std::endl << p->A << std::endl;
					//std::cout << "jacobian: " << std::endl << jacobian_i.transpose()*jacobian_j << std::endl;
					//std::cout << idx_i << " " << idx_j << " " << size_i << " " << size_j << std::endl;

					p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
					p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
				}
			}
			p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
		}
	}
	return threadsstruct;
}

void* ThreadsConstructA1(void* threadsstruct)
{
	ThreadsStruct* p = ((ThreadsStruct*)threadsstruct);
	for (const auto& it : p->sub_factors) {
		for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); ++i) {
			int idx_i = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
			int size_i = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])];
			if (size_i == 7) size_i = 6;
			Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
			for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); ++j) {
				int idx_j = p->parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
				int size_j = p->parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])];
				if (size_j == 7) size_j = 6;
				Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
				if (i == j) {
					p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
				}
				else {
					p->A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
					p->A.block(idx_j, idx_i, size_j, size_i) = p->A.block(idx_i, idx_j, size_i, size_j).transpose();
				}
			}
			p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
		}
	}
    return threadsstruct;
}

void MarginalizationInfo::marginalize()
{
	// 构造成员变量parameter_block_idx
	int pos = 0;
    for (auto &it : parameter_block_idx) {
		it.second = pos;
		pos += localSize(parameter_block_size[it.first]);
    }

	// 需要边缘化掉的矩阵维度
	m = pos;

    for (const auto &it : parameter_block_size) {
        if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
			parameter_block_idx[it.first] = pos;
			pos += localSize(it.second);
        }
    }

	// 需要保留的矩阵维度
    n = pos - m;

    TicToc t_summing;
	Eigen::MatrixXd A(pos, pos);
	Eigen::VectorXd b(pos);
	A.setZero();
	b.setZero();
    
	// 单线程构造雅可比矩阵和残差矩阵
	if (0) {
		TicToc t_sum;
		for (const auto& it : factors) {
			for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); ++i) {
				int idx_i = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
				int size_i = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
				Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
				for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); ++j) {
					int idx_j = parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])];
					int size_j = localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
					Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
					if (i == j) {
						A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
					}
					else {
						A.block(idx_i, idx_j, size_i, size_j) += jacobian_i.transpose() * jacobian_j;
						A.block(idx_j, idx_i, size_j, size_i) = A.block(idx_i, idx_j, size_i, size_j).transpose();
					}
				}
				b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
			}
		}
		//console::print_info("no thread method sum up costs: %d ms.\n", int(t_sum.toc()));
	}
	// 多线程构造雅可比矩阵和残差矩阵
	else {
		TicToc t_thread_sum;
		std::thread tids[NUM_THREADS];
		ThreadsStruct threadsstruct[NUM_THREADS];
		int i = 0;
		for (const auto& it : factors) {
			threadsstruct[i++].sub_factors.emplace_back(it);
			i = i % NUM_THREADS;
		}

		for (int i = 0; i < NUM_THREADS; ++i) {
			threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
			threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
			tids[i] = std::thread(ThreadsConstructA2, (void*)&(threadsstruct[i]),
				std::ref(parameter_block_size), std::ref(parameter_block_idx));
		}

		for (int i = NUM_THREADS - 1; i >= 0; i--) {
			tids[i].join();
			A += threadsstruct[i].A;
			b += threadsstruct[i].b;
		}
		//console::print_info("thread method sum up costs: %d ms.\n", int(t_thread_sum.toc()));
		//console::print_info("A diff %f, b diff %f.\n", (A - tmp_A).sum(), (b - tmp_b).sum());
	}

	//TicToc t_margin_inv;
	Eigen::MatrixXd Amm = 0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

    //ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f", saes.eigenvalues().minCoeff());

    Eigen::MatrixXd Amm_inv = saes.eigenvectors() * Eigen::VectorXd((saes.eigenvalues().array() > eps).select(saes.eigenvalues().array().inverse(), 0)).asDiagonal() * saes.eigenvectors().transpose();
    //printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m, m)).sum());

    Eigen::VectorXd bmm = b.segment(0, m);
    Eigen::MatrixXd Amr = A.block(0, m, m, n);	// 右上角矩阵块
    Eigen::MatrixXd Arm = A.block(m, 0, n, m);	// 左下角矩阵快
	Eigen::MatrixXd Arr = A.block(m, m, n, n);	// 右下角矩阵块
	Eigen::VectorXd brr = b.segment(m, n);
	A = Arr - Arm * Amm_inv * Amr;
	b = brr - Arm * Amm_inv * bmm;

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
	Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
	Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

	Eigen::VectorXd S_sqrt = S.cwiseSqrt();
	Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

	linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
	linearized_residuals = S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
	//console::print_info("error: %f %f.\n", (linearized_jacobians.transpose()*linearized_jacobians - A).sum(),
	//	(linearized_jacobians.transpose()*linearized_residuals - b).sum());
	//console::print_info("INFO: marginalization margin inverse: %.1f ms.\n", t_margin_inv.toc());
}

std::vector<double *> MarginalizationInfo::getParameterBlocks(std::unordered_map<long, double*> &addr_shift)
{
	std::vector<double*> keep_block_addr;
    keep_block_size.clear();
    keep_block_idx.clear();
    keep_block_data.clear();

    for (const auto &it : parameter_block_idx) {
        if (it.second >= m) {
			keep_block_size.emplace_back(parameter_block_size[it.first]);
			keep_block_idx.emplace_back(parameter_block_idx[it.first]);
			keep_block_data.emplace_back(parameter_block_data[it.first]);
			keep_block_addr.emplace_back(addr_shift[it.first]);
        }
    }
	sum_block_size = std::accumulate(std::begin(keep_block_size), std::end(keep_block_size), 0);

	return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(MarginalizationInfo* _marginalization_info)
	:marginalization_info(_marginalization_info)
{
    int cnt = 0;
    for (const auto& it : marginalization_info->keep_block_size) {
        mutable_parameter_block_sizes()->emplace_back(it);
        cnt += it;
    }
    //printf("residual size: %d, %d\n", cnt, n);
	set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    /*printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(), num_residuals());
	for (int i = 0; i < static_cast<int>(keep_block_size.size()); i++)
	{
		printf("unsigned %x\n", reinterpret_cast<unsigned long>(parameters[i]));
		printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
		printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
		printf("residual %x\n", reinterpret_cast<long>(residuals));
	}*/

	// 计算边缘化后相同优化变量不同线性化点的dx
    int n = marginalization_info->n;
    int m = marginalization_info->m;
    Eigen::VectorXd dx(n);
    for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); ++i) {
        int size = marginalization_info->keep_block_size[i];
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
        Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(marginalization_info->keep_block_data[i], size);
		if (size != 7) {
			dx.segment(idx, size) = x - x0;
		}
        else {
            dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
            dx.segment<3>(idx + 3) = 2.0 * Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).w() >= 0)) {
                dx.segment<3>(idx + 3) = 2.0 * -Utility::positify(Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() * Eigen::Quaterniond(x(6), x(3), x(4), x(5))).vec();
            }
        }
    }

	// 计算边缘化后优化变量变化后新的残差
	Eigen::Map<Eigen::VectorXd>(residuals, n) = marginalization_info->linearized_residuals + marginalization_info->linearized_jacobians * dx;
    if (jacobians) {
        for (int i = 0; i < static_cast<int>(marginalization_info->keep_block_size.size()); ++i) {
            if (jacobians[i]) {
				int size = marginalization_info->keep_block_size[i];
				int local_size = marginalization_info->localSize(size);
				int idx = marginalization_info->keep_block_idx[i] - m;
                Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> jacobian(jacobians[i], n, size);
				jacobian.setZero();
				jacobian.leftCols(local_size) = marginalization_info->linearized_jacobians.middleCols(idx, local_size);
            }
        }
    }
	return true;
}
