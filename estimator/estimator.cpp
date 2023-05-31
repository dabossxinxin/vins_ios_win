#include "estimator.h"

Estimator::Estimator(): f_manager{Rs}
{
    clearState();
    failure_occur = 0;
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; ++i) {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
	ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Eigen::Matrix2d::Identity();
}

void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; ++i){
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        if (pre_integrations[i] != nullptr){
            delete pre_integrations[i];
        }
		pre_integrations[i] = nullptr;
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
        tic[i] = Eigen::Vector3d::Zero();
        ric[i] = Eigen::Matrix3d::Identity();
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
	global_cloud.clear();			// 新增项
    initial_timestamp = 0;
    all_image_frame.clear();
    relocalize = false;
    retrive_data_vector.clear();
    relocalize_t = Eigen::Vector3d(0, 0, 0);
    relocalize_r = Eigen::Matrix3d::Identity();

    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();
}

void Estimator::processIMU(double dt, const Eigen::Vector3d &linear_acceleration, const Eigen::Vector3d &angular_velocity)
{
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count]) {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }

    if (frame_count != 0) {
		// 进来的惯导数据进入预积分环节
        pre_integrations[frame_count]->emplace_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->emplace_back(dt, linear_acceleration, angular_velocity);

		// 保存进来的惯导数据到对应buf中
        dt_buf[frame_count].emplace_back(dt);
        linear_acceleration_buf[frame_count].emplace_back(linear_acceleration);
		angular_velocity_buf[frame_count].emplace_back(angular_velocity);

		// 中值积分计算图像帧状态初始值
		int j = frame_count;
		Eigen::Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
		Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
		Eigen::Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
		Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
		Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
		Vs[j] += dt * un_acc;
    }
	acc_0 = linear_acceleration;
	gyr_0 = angular_velocity;
}

void Estimator::processImage(const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>> &image, const std_msgs::Header &header)
{
	// 将图像特征数据加入f_manage中并判断次新帧是否为关键帧
	if (f_manager.addFeatureCheckParallax(frame_count, image, td))
		marginalization_flag = MARGIN_OLD;
	else
		marginalization_flag = MARGIN_SECOND_NEW;

    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(std::make_pair(header.stamp.toSec(), imageframe));
	tmp_pre_integration = new IntegrationBase{ acc_0, gyr_0, Bas[frame_count], Bgs[frame_count] };

	// 初始化过程中得到相机与IMU之间的旋转外参
    if(ESTIMATE_EXTRINSIC == 2) {
        if (frame_count != 0) {
			Eigen::Matrix3d calib_ric;
			std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric)) {
				ric[0] = calib_ric;
				RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }

	// 进行前端滑窗优化
    if (solver_flag == INITIAL) {
		if (frame_count == WINDOW_SIZE) {
			bool result = false;
			if (ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1) {
				result = initialStructure();
				initial_timestamp = header.stamp.toSec();
			}
			if (result) {
				solver_flag = NON_LINEAR;

				TicToc t_solve;
				solveOdometry();
				console::print_info("INFO: estimator solve odometry time: %d ms.\n", int(t_solve.toc()));

				slideWindow();
				f_manager.removeFailures();
				console::print_highlight("INFO: estimator initialize success.\n");

				// 优化完成后获取滑窗中第一帧与最后一帧的位姿
				last_R = Rs[WINDOW_SIZE];
				last_P = Ps[WINDOW_SIZE];
				last_R0 = Rs[0];
				last_P0 = Ps[0];
			}
			else
				slideWindow();
		}
        else
            frame_count++;
    }
    else {
		TicToc t_solve;
		solveOdometry();
		console::print_info("INFO: estimator solve odeometry time: %d ms.\n", int(t_solve.toc()));

		// 检测系统是否有异常存在，若存在异常则重置系统
		if (failureDetection()) {
			console::print_warn("WARN: estimator failure detect.\n");
			failure_occur = 1;
			clearState();
			setParameter();
			console::print_warn("WARN: eatimator system reboot.\n");
			return;
		}

        slideWindow();
		f_manager.removeFailures();
        
		// 准备系统的输出数据
        key_poses.clear();
		for (int i = 0; i <= WINDOW_SIZE; i++)
			key_poses.emplace_back(Ps[i]);
		
		local_cloud.clear();		// 新增项
		updatePointCloud();			// 新增项

		// 优化完成后获取滑窗中第一帧与最后一帧的位姿
		last_R = Rs[WINDOW_SIZE];
		last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // 通过检查IMU数据检查系统是否有足够的运动激励
    {
		Eigen::Vector3d sum_g;
		std::map<double, ImageFrame>::iterator frame_it;
		
		// 计算IMU运动加速度均值
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
            double dt = frame_it->second.pre_integration->sum_dt;
			Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
		Eigen::Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);

		// 计算IMU运动加速度方差
        double var = 0;
		for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++) {
			double dt = frame_it->second.pre_integration->sum_dt;
			Eigen::Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
			var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
		}
        var = std::sqrt(var / ((int)all_image_frame.size() - 1));

		// 若IMU运动加速度方差不满足要求，则退出初始化
		if (var < 0.25) {
			console::print_warn("WARN: estimator IMU excitation not enouth.\n");
			marginalization_flag = MARGIN_OLD;
			return false;
		}
    }

    // 构造sfm所需的图像特征数据
	std::vector<SFMFeature> sfm_f;
    std::vector<Eigen::Quaterniond> Q(frame_count + 1);
	std::vector<Eigen::Vector3d> T(frame_count + 1);
	
    for (auto &it_per_id : f_manager.feature) {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
		tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
			Eigen::Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.emplace_back(std::make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
		sfm_f.emplace_back(tmp_feature);
    } 
	Eigen::Matrix3d relative_R;
	Eigen::Vector3d relative_T;

	// 寻找初始滑窗中与最新帧有足够视差和共视关系的帧ID
	int l;
	if (!relativePose(relative_R, relative_T, l)) {
		console::print_warn("WARN: estimator not enough features or parallax; move device around.\n");
		marginalization_flag = MARGIN_OLD;
		return false;
	}
    
	GlobalSFM sfm;
	std::map<int, Eigen::Vector3d> sfm_tracked_points;

	if (!sfm.construct(frame_count + 1, Q, T, l, relative_R,
		relative_T, sfm_f, sfm_tracked_points)) {
		console::print_warn("WARN: global SFM failed.\n");
		marginalization_flag = MARGIN_OLD;
		return false;
	}

    // 求解all_image_frame中所有帧的位置和姿态
	std::map<double, ImageFrame>::iterator frame_it;
	std::map<int, Eigen::Vector3d>::iterator it;
	frame_it = all_image_frame.begin();
	for (int i = 0; frame_it != all_image_frame.end(); frame_it++) {
        if((frame_it->first) == Headers[i].stamp.toSec()) {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
			continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec()) {
            i++;
        }

		cv::Mat r, rvec, t, D, tmp_r;
		Eigen::Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
		Eigen::Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

		frame_it->second.is_key_frame = false;
		std::vector<cv::Point3f> pts_3_vector;
		std::vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points) {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second) {
				it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end()) {
					Eigen::Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
					pts_3_vector.emplace_back(pts_3);
					Eigen::Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
					pts_2_vector.emplace_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6) {
			console::print_warn("WARN: estimator not enough points for solve pnp.\n");
			marginalization_flag = MARGIN_OLD;
			return false;
        }
		if (!cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1)) {
			console::print_warn("WARN: estimator solve pnp failed.\n");
			marginalization_flag = MARGIN_OLD;
			return false;
		}
        cv::Rodrigues(rvec, r);
		Eigen::MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
		Eigen::MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose(); // w_R_i
        frame_it->second.T = T_pnp;						 // w_T_c
    }

	// 视觉惯导联合初始化
	if (visualInitialAlign())
		return true;
    else {
		console::print_warn("WARN: estimator misalign visual structure with IMU.\n");
		marginalization_flag = MARGIN_OLD;
		return false;
    }
}

bool Estimator::visualInitialAlign()
{
	Eigen::VectorXd x;
	bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result) {
		console::print_warn("WARN: visual IMU alignment failed.\n");
        return false;
    }

	// 根据视觉-惯导对齐后的结果更新滑窗状态
    for (int i = 0; i <= frame_count; i++) {
		Eigen::Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
		Eigen::Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
		Ps[i] = Pi;
		Rs[i] = Ri;
		all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

	Eigen::VectorXd dep = f_manager.getDepthVector();
	for (int i = 0; i < dep.size(); i++)
		dep[i] = -1;
	f_manager.clearDepth(dep);

    // 三角化f_manager中管理的地图点
	// 因为没有加入尺度以及外参平移信息，因此没有尺度
	Eigen::Vector3d TIC_TMP[NUM_OF_CAM];
	for (int i = 0; i < NUM_OF_CAM; i++)
		TIC_TMP[i].setZero();
	ric[0] = RIC[0];
	f_manager.setRic(ric);
	f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++) {
		pre_integrations[i]->repropagate(Eigen::Vector3d::Zero(), Bgs[i]);
    }
	for (int i = frame_count; i >= 0; i--)
		Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
	std::map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++) {
        if(frame_i->second.is_key_frame) {
			kv++;
			Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }

	// 恢复f_manager中管理的特征点的尺度
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
		it_per_id.estimated_depth *= s;
    }

	Eigen::Matrix3d R0 = Utility::g2R(g);
	double yaw = Utility::R2ypr(R0 * Rs[0]).x();
	R0 = Utility::ypr2R(Eigen::Vector3d{ -yaw, 0, 0 }) * R0;
    g = R0 * g;
	Eigen::Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; ++i) {
		Ps[i] = rot_diff * Ps[i];
		Rs[i] = rot_diff * Rs[i];
		Vs[i] = rot_diff * Vs[i];
    }

	Eigen::Vector3d ypr = Utility::R2ypr(Rs[0]);
	console::print_highlight("convert g: %f: [%f,%f,%f].\n", g.norm(), g(0), g(1), g(2));
	console::print_highlight("convert R0: yaw-pitch-roll: [%f, %f, %f].\n", ypr(0), ypr(1), ypr(2));

	return true;
}

bool Estimator::relativePose(Eigen::Matrix3d &relative_R, Eigen::Vector3d &relative_T, int &l)
{
    // 在滑窗中寻找与最新帧有充足视差和共视点的图像帧
    for (int i = 0; i < WINDOW_SIZE; i++) {
		std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) {
			// 计算两帧特征间的平均视差
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++) {
				Eigen::Vector2d pts_0(corres[j].first(0), corres[j].first(1));
				Eigen::Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;
            }

			// 若两帧之间有充足视差且成功求解出两帧间的位姿变化则返回l
			average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T)) {
                l = i;
				console::print_info("INFO: initialize average_parallax: %d \n", int(average_parallax * 460));
				console::print_info("INFO: initialize choose l: %d, corres.size(): %d.\n", l, int(corres.size()));
                return true;
            }
        }
    }
	return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR) {
		f_manager.triangulate(Ps, tic, ric);
		optimization();
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
		Eigen::Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
		Eigen::Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

	Eigen::VectorXd dep = f_manager.getDepthVector();
	for (int i = 0; i < f_manager.getFeatureCount(); ++i)
		para_Feature[i][0] = dep(i);

	if (ESTIMATE_TD)
		para_Td[0][0] = td;
}

void Estimator::double2vector()
{
	Eigen::Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
	Eigen::Vector3d origin_P0 = Ps[0];

    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
		failure_occur = 0;
    }
	Eigen::Vector3d origin_R00 = Utility::R2ypr(
		Eigen::Quaterniond(para_Pose[0][6],
			para_Pose[0][3],
			para_Pose[0][4],
			para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
	Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d(y_diff, 0, 0));

    for (int i = 0; i <= WINDOW_SIZE; i++) {
		Rs[i] = rot_diff * Eigen::Quaterniond(
			para_Pose[i][6],
			para_Pose[i][3],
			para_Pose[i][4],
			para_Pose[i][5]).normalized().toRotationMatrix();
		Ps[i] = rot_diff * Eigen::Vector3d(
			para_Pose[i][0] - para_Pose[0][0],
			para_Pose[i][1] - para_Pose[0][1],
			para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
		Vs[i] = rot_diff * Eigen::Vector3d(
			para_SpeedBias[i][0],
			para_SpeedBias[i][1],
			para_SpeedBias[i][2]);

		Bas[i] = Eigen::Vector3d(
			para_SpeedBias[i][3],
			para_SpeedBias[i][4],
			para_SpeedBias[i][5]);

		Bgs[i] = Eigen::Vector3d(
			para_SpeedBias[i][6],
			para_SpeedBias[i][7],
			para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++) {
		tic[i] = Eigen::Vector3d(
			para_Ex_Pose[i][0],
			para_Ex_Pose[i][1],
			para_Ex_Pose[i][2]);
		ric[i] = Eigen::Quaterniond(
			para_Ex_Pose[i][6],
			para_Ex_Pose[i][3],
			para_Ex_Pose[i][4],
			para_Ex_Pose[i][5]).toRotationMatrix();
    }

	Eigen::VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
	if (ESTIMATE_TD)
		td = para_Td[0][0];

    if (LOOP_CLOSURE && relocalize && retrive_data_vector[0].relative_pose && !retrive_data_vector[0].relocalized) {
        for (int i = 0; i < (int)retrive_data_vector.size();i++)
            retrive_data_vector[i].relocalized = true;

		Eigen::Matrix3d vio_loop_r;
		Eigen::Vector3d vio_loop_t;

		vio_loop_r = rot_diff * Eigen::Quaterniond(
			retrive_data_vector[0].loop_pose[6],
			retrive_data_vector[0].loop_pose[3],
			retrive_data_vector[0].loop_pose[4],
			retrive_data_vector[0].loop_pose[5]).normalized().toRotationMatrix();
		vio_loop_t = rot_diff * Eigen::Vector3d(
			retrive_data_vector[0].loop_pose[0] - para_Pose[0][0],
			retrive_data_vector[0].loop_pose[1] - para_Pose[0][1],
			retrive_data_vector[0].loop_pose[2] - para_Pose[0][2]) + origin_P0;

		Eigen::Quaterniond vio_loop_q(vio_loop_r);
		double relocalize_yaw;
		relocalize_yaw = Utility::R2ypr(retrive_data_vector[0].R_old).x() - Utility::R2ypr(vio_loop_r).x();
		relocalize_r = Utility::ypr2R(Eigen::Vector3d(relocalize_yaw, 0, 0));
		relocalize_t = retrive_data_vector[0].P_old - relocalize_r * vio_loop_t;
    }
}

void Estimator::updatePointCloud()
{
	for (auto &it_per_id : f_manager.feature) {
		it_per_id.used_num = it_per_id.feature_per_frame.size();
		if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
			continue;

		if (it_per_id.solve_flag != 1)
			continue;

		int imu_i = it_per_id.start_frame;
		Eigen::Vector3d pt_c = it_per_id.feature_per_frame[0].point*it_per_id.estimated_depth;
		Eigen::Vector3d pt_w = relocalize_r * Rs[imu_i] * (ric[0] * pt_c + tic[0]) +
			relocalize_r * Ps[imu_i] + relocalize_t;

		global_cloud[it_per_id.feature_id] = pt_w;
		local_cloud[it_per_id.feature_id] = pt_w;
	}
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2) {
        console::print_error("little feature: %d.\n", f_manager.last_track_num);
        return true;
    }
    if (Bas[WINDOW_SIZE].norm() > 2.5) {
        console::print_error("big IMU acc bias estimation %f.\n", Bas[WINDOW_SIZE].norm());
        return true;
    }
    if (Bgs[WINDOW_SIZE].norm() > 1.0) {
        console::print_error("big IMU gyr bias estimation %f.\n", Bgs[WINDOW_SIZE].norm());
        return true;
    }
	Eigen::Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5) {
		console::print_error("big translation detect.\n");
        return true;
    }
    if (std::abs(tmp_P.z() - last_P.z()) > 1) {
        console::print_error("big z translation detect.\n");
        return true; 
    }
	Eigen::Matrix3d tmp_R = Rs[WINDOW_SIZE];
	Eigen::Matrix3d delta_R = tmp_R.transpose() * last_R;
	Eigen::Quaterniond delta_Q(delta_R);
    double delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (std::abs(delta_angle) > 50) {
        console::print_error("big delta_angle detect.\n");
        return true;
    }
    return false;
}

void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    //loss_function = new ceres::HuberLoss(1.0);
	loss_function = new ceres::CauchyLoss(1.0);

	// 添加优化量P、V、Q、Ba、Bg
    for (int i = 0; i < WINDOW_SIZE + 1; ++i) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
		problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
		problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }

	// 添加相机-IMU外参优化量
    for (int i = 0; i < NUM_OF_CAM; ++i) {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC) {
			problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
    }

	if (ESTIMATE_TD) {
		problem.AddParameterBlock(para_Td[0], 1);
	}

	TicToc t_whole, t_prepare;
	vector2double();

	// 添加边缘化残差
    if (last_marginalization_info) {
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
		problem.AddResidualBlock(marginalization_factor, NULL, last_marginalization_parameter_blocks);
    }

	// 添加IMU观测残差
    for (int i = 0; i < WINDOW_SIZE; ++i) {
		int j = i + 1;
		if (pre_integrations[j]->sum_dt > 10.0)
			continue;

        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }

	// 添加视觉残差
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature) {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
		if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
			continue;
 
        ++feature_index;

		int imu_i = it_per_id.start_frame;
		int imu_j = imu_i - 1;
        
		Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame) {
            imu_j++;
            if (imu_i == imu_j)
                continue;
            
			Eigen::Vector3d pts_j = it_per_frame.point;
			if (ESTIMATE_TD) {
				ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j,
					it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
					it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
					it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
				problem.AddResidualBlock(f_td, loss_function,
					para_Pose[imu_i], para_Pose[imu_j],
					para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
			}
			else {
				ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
				problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
			}
			f_m_cnt++;
        }
    }

	relocalize = false;

	// 添加闭环中的视觉约束
    if(LOOP_CLOSURE) {
        int loop_constraint_num = 0;
        for (int k = 0; k < (int)retrive_data_vector.size(); ++k) {    
            for(int i = 0; i < WINDOW_SIZE; i++) {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec()) {
                    relocalize = true;
                    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
                    problem.AddParameterBlock(retrive_data_vector[k].loop_pose, SIZE_POSE, local_parameterization);
                    loop_window_index = i;
					loop_constraint_num++;
                    int retrive_feature_index = 0;
                    int feature_index = -1;
                    for (auto &it_per_id : f_manager.feature) {
                        it_per_id.used_num = it_per_id.feature_per_frame.size();
						if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
							continue;

						++feature_index;
                        int start = it_per_id.start_frame;
                        if(start <= i) {   
                            while(retrive_data_vector[k].features_ids[retrive_feature_index] < it_per_id.feature_id) {
                                retrive_feature_index++;
                            }

                            if(retrive_data_vector[k].features_ids[retrive_feature_index] == it_per_id.feature_id)  {
								Eigen::Vector3d pts_j = Eigen::Vector3d(
									retrive_data_vector[k].measurements[retrive_feature_index].x,
									retrive_data_vector[k].measurements[retrive_feature_index].y, 1.0);
								Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                                
                                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                                problem.AddResidualBlock(f, loss_function, para_Pose[start], 
									retrive_data_vector[k].loop_pose, para_Ex_Pose[0], para_Feature[feature_index]);
                            
								retrive_feature_index++;
                            }     
                        }
                    }
                            
                }
            }
        }
    }

    ceres::Solver::Options options;

	options.linear_solver_type = ceres::DENSE_SCHUR;
	options.num_threads = 2;
	options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    //options.use_explicit_schur_complement = true;
    //options.minimizer_progress_to_stdout = true;
    //options.use_nonmonotonic_steps = true;
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //cout << summary.BriefReport() << endl;
	//console::print_highlight("optimize iterations: %d, cost: %.3f, solver costs: %.1f ms.\n",
	//	summary.iterations.size(), summary.final_cost,  t_solver.toc());

    // 计算优化后同一帧的相机姿态变换
    if(LOOP_CLOSURE && relocalize) { 
        for (int k = 0; k < (int)retrive_data_vector.size(); k++) {
            for(int i = 0; i< WINDOW_SIZE; i++) {
                if(retrive_data_vector[k].header == Headers[i].stamp.toSec()) {
					retrive_data_vector[k].relative_pose = true;
					Eigen::Matrix3d Rs_i = Eigen::Quaterniond(
						para_Pose[i][6], 
						para_Pose[i][3], 
						para_Pose[i][4], 
						para_Pose[i][5]).normalized().toRotationMatrix();
					Eigen::Vector3d Ps_i = Eigen::Vector3d(
						para_Pose[i][0],
						para_Pose[i][1],
						para_Pose[i][2]);

					Eigen::Matrix3d Rs_loop = Eigen::Quaterniond(
						retrive_data_vector[k].loop_pose[6], 
						retrive_data_vector[k].loop_pose[3], 
						retrive_data_vector[k].loop_pose[4], 
						retrive_data_vector[k].loop_pose[5]).normalized().toRotationMatrix();;
					Eigen::Vector3d Ps_loop = Eigen::Vector3d(
						retrive_data_vector[k].loop_pose[0],
						retrive_data_vector[k].loop_pose[1],
						retrive_data_vector[k].loop_pose[2]);

                    retrive_data_vector[k].relative_t = Rs_loop.transpose() * (Ps_i - Ps_loop);
                    retrive_data_vector[k].relative_q = Rs_loop.transpose() * Rs_i;
                    retrive_data_vector[k].relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs_i).x() - Utility::R2ypr(Rs_loop).x());

					if (std::abs(retrive_data_vector[k].relative_yaw) > 30.0 || retrive_data_vector[k].relative_t.norm() > 20.0)
						retrive_data_vector[k].relative_pose = false;
                }
            } 
        } 
    }

	double2vector();

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) {
		MarginalizationInfo *marginalization_info = new MarginalizationInfo();
		vector2double();

		// 添加上一滑窗优化过程的边缘化信息
		{
			if (last_marginalization_info) {
				std::vector<int> drop_set;
				for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
					if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
						last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
						drop_set.emplace_back(i);
				}
				// construct new marginlization_factor
				MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
				ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
					marginalization_factor, NULL,
					last_marginalization_parameter_blocks,
					drop_set);

				marginalization_info->addResidualBlockInfo(residual_block_info);
			}
		}

		// 添加最老帧IMU的边缘化信息
        {
            if (pre_integrations[1]->sum_dt < 10.0) {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
				ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
					std::vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
					std::vector<int>{0, 1});
				marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

		// 添加最老帧视觉边缘化信息
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature) {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

				// 只取第0帧观测到的特征点
				int imu_i = it_per_id.start_frame;
				int imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

				Eigen::Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame) {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

					Eigen::Vector3d pts_j = it_per_frame.point;
					if (ESTIMATE_TD) {
						ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j,
							it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
							it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
							it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
						ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
							std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
							std::vector<int>{0, 3});
						marginalization_info->addResidualBlockInfo(residual_block_info);
					}
					else {
						ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
						ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
							std::vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
							std::vector<int>{0, 3});
						marginalization_info->addResidualBlockInfo(residual_block_info);
					}
                }
            }
        }

		marginalization_info->preMarginalize();
		marginalization_info->marginalize();

		// 此处提前占位
		std::unordered_map<long, double*> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++) {
			addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
		for (int i = 0; i < NUM_OF_CAM; i++) {
			addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
		}
		if (ESTIMATE_TD) {
			addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
		}
		std::vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

		if (last_marginalization_info)
			delete last_marginalization_info;

		last_marginalization_info = std::move(marginalization_info);
		last_marginalization_parameter_blocks = std::move(parameter_blocks);
    }
    else {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {
			MarginalizationInfo *marginalization_info = new MarginalizationInfo();
			vector2double();

			// 添加上一滑窗优化过程中的边缘化信息
			{
				if (last_marginalization_info) {
					std::vector<int> drop_set;
					for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++) {
						assert(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
						if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
							drop_set.emplace_back(i);
					}
					// construct new marginlization_factor
					MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
					ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
						marginalization_factor, NULL,
						last_marginalization_parameter_blocks,
						drop_set);

					marginalization_info->addResidualBlockInfo(residual_block_info);
				}
			}
            
            marginalization_info->preMarginalize();
			marginalization_info->marginalize();
            
			// 此处提前占位
			std::unordered_map<long, double*> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; ++i) {
				if (i == WINDOW_SIZE - 1)
					continue;
                else if (i == WINDOW_SIZE) {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
			for (int i = 0; i < NUM_OF_CAM; i++) {
				addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
			}
			if (ESTIMATE_TD) {
				addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
			}

			std::vector<double*> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
			last_marginalization_info = std::move(marginalization_info);
			last_marginalization_parameter_blocks = std::move(parameter_blocks);
        }
    }
}

void Estimator::slideWindow()
{
    if (marginalization_flag == MARGIN_OLD) {
		back_R0 = Rs[0];
		back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE) {
            for (int i = 0; i < WINDOW_SIZE; ++i) {
				Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

				dt_buf[i].swap(dt_buf[i + 1]);
				linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
				angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL) {
                double t_0 = Headers[0].stamp.toSec();
				std::map<double, ImageFrame>::iterator it_0;
				it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
				all_image_frame.erase(all_image_frame.begin(), it_0);
            }
			slideWindowOld();
        }
    }
    else {
        if (frame_count == WINDOW_SIZE) {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); ++i) {
                double tmp_dt = dt_buf[frame_count][i];
				Eigen::Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
				Eigen::Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->emplace_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

				dt_buf[frame_count - 1].emplace_back(tmp_dt);
				linear_acceleration_buf[frame_count - 1].emplace_back(tmp_linear_acceleration);
				angular_velocity_buf[frame_count - 1].emplace_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
			Bgs[frame_count - 1] = Bgs[frame_count];

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
	f_manager.removeFront(frame_count);
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

	bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth) {
		Eigen::Matrix3d R0, R1;
		Eigen::Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
		f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}