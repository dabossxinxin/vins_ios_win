#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <ceres/ceres.h>

#include "utility/utility.h"
#include "utility/print.h"
#include "parameters.h"
#include "integration_base.h"

class IMUFactor : public ceres::SizedCostFunction<15, 7, 9, 7, 9>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IMUFactor() = delete;
	IMUFactor(IntegrationBase* _pre_integration) :pre_integration(_pre_integration)
	{
	}
	virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
	{
		Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
		Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

		Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
		Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
		Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

		Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
		Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

		Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
		Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
		Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

		//Eigen::Matrix<double, 15, 15> Fd;
		//Eigen::Matrix<double, 15, 12> Gd;

		//Eigen::Vector3d pPj = Pi + Vi * sum_t - 0.5 * g * sum_t * sum_t + corrected_delta_p;
		//Eigen::Quaterniond pQj = Qi * delta_q;
		//Eigen::Vector3d pVj = Vi - g * sum_t + corrected_delta_v;
		//Eigen::Vector3d pBaj = Bai;
		//Eigen::Vector3d pBgj = Bgi;

		//Vi + Qi * delta_v - g * sum_dt = Vj;
		//Qi * delta_q = Qj;

		//delta_p = Qi.inverse() * (0.5 * g * sum_dt * sum_dt + Pj - Pi);
		//delta_v = Qi.inverse() * (g * sum_dt + Vj - Vi);
		//delta_q = Qi.inverse() * Qj;

#if 0
		if ((Bai - pre_integration->linearized_ba).norm() > 0.10 ||
			(Bgi - pre_integration->linearized_bg).norm() > 0.01)
		{
			pre_integration->repropagate(Bai, Bgi);
		}
#endif

		Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
		residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
			Pj, Qj, Vj, Baj, Bgj);

		Eigen::Matrix<double, 15, 15> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(pre_integration->covariance.inverse()).matrixL().transpose();
		residual = sqrt_info * residual;

		if (jacobians)
		{
			double sum_dt = pre_integration->sum_dt;
			Eigen::Matrix3d dp_dba = pre_integration->jacobian.template block<3, 3>(O_P, O_BA);
			Eigen::Matrix3d dp_dbg = pre_integration->jacobian.template block<3, 3>(O_P, O_BG);

			Eigen::Matrix3d dq_dbg = pre_integration->jacobian.template block<3, 3>(O_R, O_BG);

			Eigen::Matrix3d dv_dba = pre_integration->jacobian.template block<3, 3>(O_V, O_BA);
			Eigen::Matrix3d dv_dbg = pre_integration->jacobian.template block<3, 3>(O_V, O_BG);

			if (pre_integration->jacobian.maxCoeff() > 1e8 || pre_integration->jacobian.minCoeff() < -1e8)
			{
				console::print_warn("WARN: numerical unstable in preintegration.\n");
			}

			if (jacobians[0])
			{
				Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
				jacobian_pose_i.setZero();

				jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
				jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * G * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
				jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
				Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
				jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
#endif

				jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (G * sum_dt + Vj - Vi));

				jacobian_pose_i = sqrt_info * jacobian_pose_i;

				if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
				{
					console::print_warn("WARN: numerical unstable in preintegration.\n");
				}
			}
			if (jacobians[1])
			{
				Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
				jacobian_speedbias_i.setZero();
				jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
				jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
				jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
				jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
				Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
				jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
#endif

				jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
				jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
				jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

				jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

				jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

				jacobian_speedbias_i = sqrt_info * jacobian_speedbias_i;

				if (jacobian_speedbias_i.maxCoeff() > 1e8 || jacobian_speedbias_i.minCoeff() < -1e8)
				{
					console::print_warn("WARN: numerical unstable in preintegration.\n");
				}
			}
			if (jacobians[2])
			{
				Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
				jacobian_pose_j.setZero();

				jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
				jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
				Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
				jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();
#endif
				jacobian_pose_j = sqrt_info * jacobian_pose_j;

				if (jacobian_pose_j.maxCoeff() > 1e8 || jacobian_pose_j.minCoeff() < -1e8)
				{
					console::print_warn("WARN: numerical unstable in preintegration.\n");
				}
			}
			if (jacobians[3])
			{
				Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
				jacobian_speedbias_j.setZero();

				jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

				jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

				jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

				jacobian_speedbias_j = sqrt_info * jacobian_speedbias_j;

				if (jacobian_speedbias_j.maxCoeff() > 1e8 || jacobian_speedbias_j.minCoeff() < -1e8)
				{
					console::print_warn("WARN: numerical unstable in preintegration.\n");
				}
			}
					
			// check jacobian
			if (0)
			{
				Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
				Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);

				Eigen::Matrix<double, 15, 15> numericalJacobianI;
				Eigen::Matrix<double, 15, 15> analyticJacobianI;

				analyticJacobianI.block<15, 6>(0, 0) = jacobian_pose_i.leftCols<6>();
				analyticJacobianI.block<15, 9>(0, 6) = jacobian_speedbias_i.leftCols<9>();

				const double eps = 1e-3;

				for (int k = 0; k < 15; ++k) {
					int a = k / 3;
					int b = k % 3;
					Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2)*eps;

					Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
					Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

					Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
					Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
					Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

					Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
					Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

					Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
					Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
					Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

					if (a == 0)
						Pi += delta;
					else if (a == 1)
						Qi = Qi * Utility::deltaQ(delta);
					else if (a == 2)
						Vi += delta;
					else if (a == 3)
						Bai += delta;
					else if (a == 4)
						Bgi += delta;

					Qi.normalize();

					Eigen::Matrix<double, 15, 1> tmp_residual;
					tmp_residual = pre_integration->evaluate(Pi, Qi, Vi, Bai, Bgi,
						Pj, Qj, Vj, Baj, Bgj);

					tmp_residual = sqrt_info * tmp_residual;
					numericalJacobianI.col(k) = (tmp_residual - residual) / eps;
				}

				std::string filename = "C:\\Users\\jackchen\\Desktop\\imu_jacobians.txt";
				std::ofstream foutC(filename.c_str(), std::ios::app);
				foutC.setf(std::ios::fixed, std::ios::floatfield);
				foutC.precision(8);
				foutC << "analytic jacobians:" << std::endl;
				for (int row = 0; row < 15; ++row) {
					for (int col = 0; col < 15; ++col) {
						foutC << analyticJacobianI(row, col) << " ";
					}
					foutC << std::endl;
				}
				foutC << "numerical jacobians:" << std::endl;
				for (int row = 15; row < 15; ++row) {
					for (int col = 15; col < 15; ++col) {
						foutC << numericalJacobianI(row, col) << " ";
					}
					foutC << std::endl;
				}
				foutC.close();
			}
		}
		return true;
	}

	//bool Evaluate_Direct(double const *const *parameters, Eigen::Matrix<double, 15, 1> &residuals, Eigen::Matrix<double, 15, 30> &jacobians);
	//void checkCorrection();
	//void checkTransition();
	//void checkJacobian(double **parameters);
	IntegrationBase* pre_integration;
};