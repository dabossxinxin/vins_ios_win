#include "initial_sfm.h"
#include "../utility/print.h"

GlobalSFM::GlobalSFM(){}

void GlobalSFM::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
	Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
	Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
	design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
	design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
	design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
	design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
	Eigen::Vector4d triangulated_point;
	triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	point_3d(0) = triangulated_point(0) / triangulated_point(3);
	point_3d(1) = triangulated_point(1) / triangulated_point(3);
	point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

bool GlobalSFM::solveFrameByPnP(Eigen::Matrix3d &R_initial, Eigen::Vector3d &P_initial,
	int i, std::vector<SFMFeature> &sfm_f)
{
	// 获取2D-3D特征匹配点
	std::vector<cv::Point2f> pts_2_vector;
	std::vector<cv::Point3f> pts_3_vector;
	for (int j = 0; j < feature_num; ++j) {
		if (sfm_f[j].state != true)
			continue;

		Eigen::Vector2d point2d;
		for (int k = 0; k < (int)sfm_f[j].observation.size(); ++k) {
			if (sfm_f[j].observation[k].first == i) {
				Eigen::Vector2d img_pts = sfm_f[j].observation[k].second;
				cv::Point2f pts_2(img_pts(0), img_pts(1));
				pts_2_vector.emplace_back(pts_2);
				cv::Point3f pts_3(sfm_f[j].position[0], sfm_f[j].position[1], sfm_f[j].position[2]);
				pts_3_vector.emplace_back(pts_3);
				break;
			}
		}
	}

	if (int(pts_2_vector.size()) < 15) {
		console::print_info("INFO: initial_sfm unstable features tracking, please slowly move your device.\n");
		if (int(pts_2_vector.size()) < 10)
			return false;
	}

	// pnp求解相机位姿
	cv::Mat r, rvec, t, D, tmp_r;
	cv::eigen2cv(R_initial, tmp_r);
	cv::Rodrigues(tmp_r, rvec);
	cv::eigen2cv(P_initial, t);
	cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
	bool pnp_succ = cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1);
	if (!pnp_succ) {
		return false;
	}

	cv::Rodrigues(rvec, r);
	Eigen::MatrixXd R_pnp;
	cv::cv2eigen(r, R_pnp);
	Eigen::MatrixXd T_pnp;
	cv::cv2eigen(t, T_pnp);
	R_initial = R_pnp;
	P_initial = T_pnp;
	return true;
}

void GlobalSFM::triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0,
	int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
	std::vector<SFMFeature> &sfm_f)
{
	if (frame0 == frame1)
		return;

	for (int j = 0; j < feature_num; j++) {
		if (sfm_f[j].state == true)
			continue;
		bool has_0 = false;
		bool has_1 = false;
		Eigen::Vector2d point0;
		Eigen::Vector2d point1;
		for (int k = 0; k < (int)sfm_f[j].observation.size(); k++) {
			if (sfm_f[j].observation[k].first == frame0) {
				point0 = sfm_f[j].observation[k].second;
				has_0 = true;
			}
			if (sfm_f[j].observation[k].first == frame1) {
				point1 = sfm_f[j].observation[k].second;
				has_1 = true;
			}
		}

		if (has_0 && has_1) {
			Eigen::Vector3d point_3d;
			triangulatePoint(Pose0, Pose1, point0, point1, point_3d);
			sfm_f[j].state = true;
			sfm_f[j].position[0] = point_3d(0);
			sfm_f[j].position[1] = point_3d(1);
			sfm_f[j].position[2] = point_3d(2);
		}
	}
}

bool GlobalSFM::construct(int frame_num, std::vector<Eigen::Quaterniond>& q,
	std::vector<Eigen::Vector3d>& T, int l, const Eigen::Matrix3d relative_R, const Eigen::Vector3d relative_T,
	std::vector<SFMFeature> &sfm_f, std::map<int, Eigen::Vector3d> &sfm_tracked_points)
{
	feature_num = sfm_f.size();

	// 设定l帧与最新帧的位姿
	q[l].w() = 1;
	q[l].x() = 0;
	q[l].y() = 0;
	q[l].z() = 0;
	T[l].setZero();
	q[frame_num - 1] = q[l] * Eigen::Quaterniond(relative_R);
	T[frame_num - 1] = relative_T;

	std::vector<Eigen::Matrix3d> c_Rotation(frame_num);
	std::vector<Eigen::Vector3d> c_Translation(frame_num);
	std::vector<Eigen::Quaterniond> c_Quat(frame_num);
	std::vector<Eigen::Vector4d> c_rotation(frame_num);
	std::vector<Eigen::Vector3d> c_translation(frame_num);
	std::vector<Eigen::Matrix<double, 3, 4>> Pose(frame_num);

	// 计算第l帧在相机坐标系中的位姿
	c_Quat[l] = q[l].inverse();
	c_Rotation[l] = c_Quat[l].toRotationMatrix();
	c_Translation[l] = -1 * (c_Rotation[l] * T[l]);
	Pose[l].block<3, 3>(0, 0) = c_Rotation[l];
	Pose[l].block<3, 1>(0, 3) = c_Translation[l];

	// 计算最新帧在相机坐标系中的位姿
	c_Quat[frame_num - 1] = q[frame_num - 1].inverse();
	c_Rotation[frame_num - 1] = c_Quat[frame_num - 1].toRotationMatrix();
	c_Translation[frame_num - 1] = -1 * (c_Rotation[frame_num - 1] * T[frame_num - 1]);
	Pose[frame_num - 1].block<3, 3>(0, 0) = c_Rotation[frame_num - 1];
	Pose[frame_num - 1].block<3, 1>(0, 3) = c_Translation[frame_num - 1];

	// 恢复l~frame_num帧的相机姿态以及路标点坐标
	for (int i = l; i < frame_num - 1; ++i) {
		if (i > l) {
			Eigen::Matrix3d R_initial = c_Rotation[i - 1];
			Eigen::Vector3d P_initial = c_Translation[i - 1];
			if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)) {
				console::print_warn("WARN: initial_sfm solvePnP failed.\n");
				return false;
			}
				

			c_Rotation[i] = R_initial;
			c_Translation[i] = P_initial;
			c_Quat[i] = c_Rotation[i];
			Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
			Pose[i].block<3, 1>(0, 3) = c_Translation[i];
		}

		triangulateTwoFrames(i, Pose[i], frame_num - 1, Pose[frame_num - 1], sfm_f);
	}

	for (int i = l + 1; i < frame_num - 1; i++)
		triangulateTwoFrames(l, Pose[l], i, Pose[i], sfm_f);

	// 恢复0~l-1帧的相机姿态以及路标点坐标
	for (int i = l - 1; i >= 0; --i) {
		Eigen::Matrix3d R_initial = c_Rotation[i + 1];
		Eigen::Vector3d P_initial = c_Translation[i + 1];
		if (!solveFrameByPnP(R_initial, P_initial, i, sfm_f)) {
			console::print_warn("WARN: initial_sfm solvePnP failed.\n");
			return false;
		}

		c_Rotation[i] = R_initial;
		c_Translation[i] = P_initial;
		c_Quat[i] = c_Rotation[i];
		Pose[i].block<3, 3>(0, 0) = c_Rotation[i];
		Pose[i].block<3, 1>(0, 3) = c_Translation[i];

		triangulateTwoFrames(i, Pose[i], l, Pose[l], sfm_f);
	}

	// 三角化所有已有的路标点
	for (int j = 0; j < feature_num; j++) {
		if (sfm_f[j].state == true)
			continue;

		if ((int)sfm_f[j].observation.size() >= 2) {
			Eigen::Vector2d point0, point1;
			int frame_0 = sfm_f[j].observation[0].first;
			point0 = sfm_f[j].observation[0].second;
			int frame_1 = sfm_f[j].observation.back().first;
			point1 = sfm_f[j].observation.back().second;

			Eigen::Vector3d point_3d;
			triangulatePoint(Pose[frame_0], Pose[frame_1], point0, point1, point_3d);

			sfm_f[j].state = true;
			sfm_f[j].position[0] = point_3d(0);
			sfm_f[j].position[1] = point_3d(1);
			sfm_f[j].position[2] = point_3d(2);
		}
	}

	ceres::Problem problem;
	ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

	// 向优化问题中添加优化变量
	for (int i = 0; i < frame_num; ++i) {
		c_translation[i][0] = c_Translation[i].x();
		c_translation[i][1] = c_Translation[i].y();
		c_translation[i][2] = c_Translation[i].z();
		c_rotation[i][0] = c_Quat[i].w();
		c_rotation[i][1] = c_Quat[i].x();
		c_rotation[i][2] = c_Quat[i].y();
		c_rotation[i][3] = c_Quat[i].z();

		problem.AddParameterBlock(c_rotation[i].data(), 4, local_parameterization);
		problem.AddParameterBlock(c_translation[i].data(), 3);

		if (i == l) {
			problem.SetParameterBlockConstant(c_rotation[i].data());
		}
		if (i == l || i == frame_num - 1) {
			problem.SetParameterBlockConstant(c_translation[i].data());
		}
	}

	// 向优化问题中添加优化残差
	for (int i = 0; i < feature_num; ++i) {
		if (sfm_f[i].state != true)
			continue;

		problem.AddParameterBlock(sfm_f[i].position, 3);

		for (int j = 0; j < int(sfm_f[i].observation.size()); j++) {
			int index = sfm_f[i].observation[j].first;
			ceres::CostFunction* cost_function = ReprojectionError3D::Create(
				sfm_f[i].observation[j].second.x(),
				sfm_f[i].observation[j].second.y());

			problem.AddResidualBlock(cost_function, NULL, c_rotation[index].data(), c_translation[index].data(),
				sfm_f[i].position);
		}
	}

	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	//options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = 0.2;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";
	if (summary.termination_type == ceres::CONVERGENCE || summary.final_cost < 5e-03) {
		console::print_info("INFO: initial sfm vision only BA converge.\n");
	}
	else {
		console::print_warn("WARN: initial sfm vision only BA not converge.\n");
		return false;
	}

	// 将所有帧的姿态转为世界坐标系下的姿态
	for (int i = 0; i < frame_num; ++i) {
		q[i].w() = c_rotation[i][0];
		q[i].x() = c_rotation[i][1];
		q[i].y() = c_rotation[i][2];
		q[i].z() = c_rotation[i][3];
		q[i] = q[i].inverse();
	}

	// 将所有帧的位置转化为世界坐标系的位置
	for (int i = 0; i < frame_num; ++i) {
		T[i] = -1 * (q[i] * Eigen::Vector3d(c_translation[i][0], c_translation[i][1], c_translation[i][2]));
	}

	// TODO: 此处的路标点并没有被优化，是否需要优化
	for (int i = 0; i < (int)sfm_f.size(); ++i) {
		if (sfm_f[i].state)
			sfm_tracked_points[sfm_f[i].id] = Eigen::Vector3d(sfm_f[i].position[0], sfm_f[i].position[1], sfm_f[i].position[2]);
	}
	return true;
}