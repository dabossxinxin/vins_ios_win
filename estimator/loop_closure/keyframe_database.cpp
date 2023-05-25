#include "keyframe_database.h"

KeyFrameDatabase::KeyFrameDatabase()
{
    //posegraph_visualization = new CameraPoseVisualization(0.0, 0.0, 1.0, 1.0);
    //posegraph_visualization->setScale(0.1);
    //posegraph_visualization->setLineWidth(0.01);
	yaw_drift = 0;
	total_length = 0;
	earliest_loop_index = -1;
	newest_keyframe_index = -1;
	last_P = Eigen::Vector3d(0, 0, 0);
    t_drift = Eigen::Vector3d(0, 0, 0);
	r_drift = Eigen::Matrix3d::Identity();
}

bool KeyFrameDatabase::viewNewestKeyFrameEx(Eigen::Vector3d& ypr, Eigen::Vector3d& tic)
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	if (!keyFrameList.empty()) {
		KeyFrame* newestKeyFrame = keyFrameList.back();
		if (newest_keyframe_index == -1 || newest_keyframe_index != newestKeyFrame->global_index) {
			tic = newestKeyFrame->tic;
			ypr = Utility::R2ypr(newestKeyFrame->qic);
			newest_keyframe_index = newestKeyFrame->global_index;
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}
}

void KeyFrameDatabase::viewPointClouds()
{
	//std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	//GLfloat size = 1.0;
	//for (auto keyframe = keyFrameList.begin(); keyframe != keyFrameList.end(); ++keyframe)
	//{
	//	glPointSize(size);
	//	glBegin(GL_POINTS);
	//	glColor3f(1.0, 1.0, 1.0);
	
	//	for (auto point = (*keyframe)->point_clouds.begin(); point != (*keyframe)->point_clouds.end(); ++point)
	//	{
	//		Eigen::Vector3d point_world = (*keyframe)->relocalize_r * (*point) + (*keyframe)->relocalize_t;
	//		glVertex3f((float)(point_world.x()), (float)(point_world.y()), (float)(point_world.z()));
	//	}
	//	glEnd();
	//}
}

void KeyFrameDatabase::viewPath()
{
	//std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	//GLfloat line_width = 1.2;
	//Eigen::Vector3d tmp_path;
	//glColor3f(1.0f, 0.0f, 0.0f);
	//glLineWidth(line_width);
	//glBegin(GL_LINE_STRIP);
	
	//for(auto it = keyFrameList.begin(); it != keyFrameList.end(); ++it) {
	//	(*it)->getPath(tmp_path);
	//	glVertex3f(tmp_path.x(), tmp_path.y(), tmp_path.z());
	//}
	//glEnd();
}

void KeyFrameDatabase::add(KeyFrame* pKF)
{
	console::print_info("INFO: add keyframe begin.\n");
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	keyFrameList.emplace_back(pKF);
	lock.unlock();

	Eigen::Vector3d P;
	Eigen::Matrix3d R;
	pKF->getPose(P, R);
	Eigen::Quaterniond Q;
	Q = R;

	total_length += (P - last_P).norm();
	last_P = P;

	//draw local connection
	/*posegraph_visualization->add_pose(P, Q);
	std::list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	std::list<KeyFrame*>::reverse_iterator lrit;
	for (; rit != keyFrameList.rend(); rit++)  
    {  
        if ((*rit) == pKF)
        {
        	lrit = rit;
        	lrit++;
        	for (int i = 0; i < 4; i++)
        	{
        		if (lrit == keyFrameList.rend())
        			break;
        		Eigen::Vector3d conncected_P;
        		Eigen::Matrix3d connected_R;
        		(*lrit)->getPose(conncected_P, connected_R);
        		posegraph_visualization->add_edge(P, conncected_P);
        		lrit++;
        	}
        	break;
        }
    } */

	// add key frame to path for visualization
	nav_msgs::Odometry odometry;
	odometry.header.stamp = ros::Time(pKF->header);
	odometry.header.frame_id = "world";
	odometry.pose.pose.position.x = P.x();
	odometry.pose.pose.position.y = P.y();
	odometry.pose.pose.position.z = P.z();
	odometry.pose.pose.orientation.x = Q.x();
	odometry.pose.pose.orientation.y = Q.y();
	odometry.pose.pose.orientation.z = Q.z();
	odometry.pose.pose.orientation.w = Q.w();

	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header = odometry.header;
	pose_stamped.pose = odometry.pose.pose;

	// TODO: 这里要不要加上目前存疑
	/*std::unique_lock<std::mutex> mPath(mPath);
	refine_path.header = odometry.header;
	refine_path.poses.emplace_back(pose_stamped);
	mPath.unlock();*/
	console::print_info("INFO: add keyframe end.\n");
}

void KeyFrameDatabase::downsample(std::vector<int> &erase_index)
{
	console::print_info("INFO: resample keyframe begin.\n");
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	int keyframe_num = (int)keyFrameList.size();
	if (mOptimiazationPosegraph.try_lock())
	{
		erase_index.clear();
		double min_dis = total_length / (keyframe_num * 0.7);

		std::list<KeyFrame*>::iterator it = keyFrameList.begin();
		Eigen::Vector3d last_P = Eigen::Vector3d(0, 0, 0);
		for (; it != keyFrameList.end(); )   
		{
			Eigen::Vector3d tmp_t;
			Eigen::Matrix3d tmp_r;
			(*it)->getPose(tmp_t, tmp_r);
			double dis = (tmp_t - last_P).norm();

			// 关键帧中的第一帧/运动距离大于最小距离/当前帧存在闭环
		    if(it == keyFrameList.begin() || dis > min_dis || (*it)->has_loop || (*it)->is_looped) {
		    	last_P = tmp_t;
		    	it++;
		    }
		    else {
				erase_index.emplace_back((*it)->global_index);
				delete (*it);
				it = keyFrameList.erase(it);
		    }
		}
		mOptimiazationPosegraph.unlock();
	}
	else 
	{
		return;
	}

	lock.unlock();
	console::print_info("INFO: resample keyframe end.\n");
}

void KeyFrameDatabase::erase(KeyFrame* pKF)
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	std::list<KeyFrame*>::iterator it = 
		std::find(keyFrameList.begin(), keyFrameList.end(), pKF);

	if (it != keyFrameList.end()) 
	{
		keyFrameList.erase(it);
	}    
}

int KeyFrameDatabase::size()
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	return (int)keyFrameList.size();
}

void KeyFrameDatabase::getKeyframeIndexList(std::vector<int> &keyframe_index_list)
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	std::list<KeyFrame*>::iterator it = keyFrameList.begin();
	for (; it != keyFrameList.end(); ++it) {
		keyframe_index_list.emplace_back((*it)->global_index);
	}
	return;
}

KeyFrame* KeyFrameDatabase::getKeyframe(int index)
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	std::list<KeyFrame*>::iterator it = keyFrameList.begin();
	for (; it != keyFrameList.end(); ++it) {
		if ((*it)->global_index == index) {
			break;
		}
	}
	if (it != keyFrameList.end()) {
		return *it;
	}
	else {
		return NULL;
	}
}

KeyFrame* KeyFrameDatabase::getLastKeyframe()
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	std::list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	if (rit != keyFrameList.rend()) {
		return *rit;
	}
	else {
		return NULL;
	}
}

KeyFrame* KeyFrameDatabase::getLastKeyframe(int last_index)
{
	std::unique_lock<std::mutex> lock(mMutexkeyFrameList);
	std::list<KeyFrame*>::reverse_iterator rit = keyFrameList.rbegin();
	for (int i = 0; i < last_index; ++i) {
		rit++;
		if (rit == keyFrameList.rend()) {
			return NULL;
		}
	}
	return *rit;
}

void KeyFrameDatabase::optimize4DoFLoopPoseGraph(int cur_index, Eigen::Vector3d &loop_correct_t, Eigen::Matrix3d &loop_correct_r)
{
	console::print_info("INFO: optimize pose graph begin.\n");
	std::unique_lock<std::mutex> lock(mOptimiazationPosegraph);
	KeyFrame* cur_kf = getKeyframe(cur_index);
	int loop_index = cur_kf->loop_index;
	if (earliest_loop_index > loop_index || earliest_loop_index == -1)
		earliest_loop_index = loop_index;
	assert(cur_kf->update_loop_info == 1);
	int max_length = cur_index + 1;

	// w^t_i   w^q_i
	std::vector<Eigen::Vector3d> t_array(max_length);
	std::vector<Eigen::Quaterniond> q_array(max_length);
	std::vector<Eigen::Vector3d> euler_array(max_length);

	ceres::Problem problem;
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	//options.minimizer_progress_to_stdout = true;
	options.max_solver_time_in_seconds = SOLVER_TIME * 3;
	options.max_num_iterations = 5;
	ceres::Solver::Summary summary;
	ceres::LossFunction *loss_function;
	loss_function = new ceres::HuberLoss(1.0);
	//loss_function = new ceres::CauchyLoss(1.0);
	ceres::LocalParameterization* angle_local_parameterization =
		AngleLocalParameterization::Create();

	int i = 0;
	for (auto it = keyFrameList.begin(); it != keyFrameList.end(); ++it)
	{
		if ((*it)->global_index < earliest_loop_index) {
			continue;
		}

		(*it)->resample_index = i;
		Eigen::Quaterniond tmp_q;
		Eigen::Matrix3d tmp_r;
		Eigen::Vector3d tmp_t;
		(*it)->getOriginPose(tmp_t, tmp_r);
		tmp_q = tmp_r;
		t_array[i][0] = tmp_t(0);
		t_array[i][1] = tmp_t(1);
		t_array[i][2] = tmp_t(2);
		q_array[i] = tmp_q;

		Eigen::Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
		euler_array[i][0] = euler_angle.x();
		euler_array[i][1] = euler_angle.y();
		euler_array[i][2] = euler_angle.z();

		problem.AddParameterBlock(euler_array[i].data(), 1, angle_local_parameterization);
		problem.AddParameterBlock(t_array[i].data(), 3);

		if ((*it)->global_index == earliest_loop_index)
		{
			problem.SetParameterBlockConstant(euler_array[i].data());
			problem.SetParameterBlockConstant(t_array[i].data());
		}

		// 添加关键帧每一帧与之前四帧之间的闭环残差
		for (int j = 1; j < 5; j++)
		{
			if (i - j > 0)
			{
				Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[i - j].toRotationMatrix());
				Eigen::Vector3d relative_t(t_array[i][0] - t_array[i - j][0], t_array[i][1] - t_array[i - j][1], t_array[i][2] - t_array[i - j][2]);
				relative_t = q_array[i - j].inverse() * relative_t;
				double relative_yaw = euler_array[i][0] - euler_array[i - j][0];
				ceres::CostFunction* cost_function = FourDOFError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
					relative_yaw, euler_conncected.y(), euler_conncected.z());
				problem.AddResidualBlock(cost_function, NULL,
					euler_array[i - j].data(),
					t_array[i - j].data(),
					euler_array[i].data(),
					t_array[i].data());
			}
		}

		// 添加当前帧与其对应闭环帧的残差
		if ((*it)->update_loop_info)
		{
			int connected_index = getKeyframe((*it)->loop_index)->resample_index;
			assert((*it)->loop_index >= earliest_loop_index);
			Eigen::Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
			Eigen::Vector3d relative_t = (*it)->getLoopRelativeT();
			double relative_yaw = (*it)->getLoopRelativeYaw();
			ceres::CostFunction* cost_function = FourDOFWeightError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
				relative_yaw, euler_conncected.y(), euler_conncected.z());
			problem.AddResidualBlock(cost_function, loss_function,
				euler_array[connected_index].data(),
				t_array[connected_index].data(),
				euler_array[i].data(),
				t_array[i].data());
		}

		if ((*it)->global_index == cur_index) {
			break;
		}
		i++;
	}

	ceres::Solve(options, &problem, &summary);
	//std::cout << summary.BriefReport() << "\n";

	i = 0;
	for (auto it = keyFrameList.begin(); it != keyFrameList.end(); it++)
	{
		if ((*it)->global_index < earliest_loop_index) {
			continue;
		}

		Eigen::Quaterniond tmp_q;
		tmp_q = Utility::ypr2R(Eigen::Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
		Eigen::Vector3d tmp_t = Eigen::Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
		Eigen::Matrix3d tmp_r = tmp_q.toRotationMatrix();
		(*it)->updatePose(tmp_t, tmp_r);

		if ((*it)->global_index == cur_index) {
			break;
		}
		i++;
	}

	Eigen::Vector3d cur_t, origin_t;
	Eigen::Matrix3d cur_r, origin_r;
	cur_kf->getPose(cur_t, cur_r);
	cur_kf->getOriginPose(origin_t, origin_r);
	yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(origin_r).x();
	r_drift = Utility::ypr2R(Eigen::Vector3d(yaw_drift, 0, 0));
	t_drift = cur_t - r_drift * origin_t;

	// TODO：使用当前闭环帧的drift对所有关键帧位姿进行矫正是否不妥
	for (auto it = keyFrameList.begin(); it != keyFrameList.end(); ++it)
	{
		Eigen::Vector3d P;
		Eigen::Matrix3d R;
		(*it)->getOriginPose(P, R);
		P = r_drift * P + t_drift;
		R = r_drift * R;
		(*it)->updatePose(P, R);
	}
	loop_correct_t = t_drift;
	loop_correct_r = r_drift;
	console::print_info("INFO: optimize pose graph end.\n");
}

void KeyFrameDatabase::updateVisualization()
{
	console::print_info("INFO: update visualization begin.\n");
	std::unique_lock<std::mutex> mlockPath(mPath);
	std::unique_lock<std::mutex> mlockPosegraph(mPosegraphVisualization);
	total_length = 0;
	last_P = Eigen::Vector3d(0, 0, 0);
	//update visualization
	std::list<KeyFrame*>::iterator it;
    //posegraph_visualization->reset();
	refine_path.poses.clear();

	for (it = keyFrameList.begin(); it != keyFrameList.end(); it++)
	{
		Eigen::Vector3d P;
		Eigen::Matrix3d R;
		(*it)->getPose(P, R);
		Eigen::Quaterniond Q;
		Q = R;

		total_length += (P - last_P).norm();
		last_P = P;

		/*posegraph_visualization->add_pose(P, Q);
		std::list<KeyFrame*>::iterator lit;
		lit = it;
    	for (int i = 0; i < 4; i++)
    	{
    		if (lit == keyFrameList.begin())
    			break;
    		lit--;
    		Eigen::Vector3d conncected_P;
    		Eigen::Matrix3d connected_R;
    		(*lit)->getPose(conncected_P, connected_R);
    		posegraph_visualization->add_edge(P, conncected_P);
    	}*/
		
		// draw loop edge
		if ((*it)->update_loop_info)
		{
			KeyFrame* connected_KF = getKeyframe((*it)->loop_index);
			Eigen::Vector3d conncected_P;
			Eigen::Matrix3d connected_R;
			connected_KF->getPose(conncected_P, connected_R);

			/*posegraph_visualization->add_loopedge(P, conncected_P);
			Eigen::Vector3d supposed_P;
			Eigen::Vector3d relative_t;
			relative_t = (*it)->getLoopRelativeT();
			supposed_P = P - connected_R * relative_t;
			posegraph_visualization->add_edge(P, supposed_P);*/	

			std::list<KeyFrame*>::iterator lit;
			lit = it;
			lit--;
			Eigen::Vector3d P_previous;
			Eigen::Matrix3d R_previous;
			(*lit)->getPose(P_previous, R_previous);
			//posegraph_visualization->add_loopedge(P, P_previous);
		}

		// add key frame to path for visualization
		nav_msgs::Odometry odometry;
		odometry.header.stamp = ros::Time((*it)->header);
		odometry.header.frame_id = "world";
		odometry.pose.pose.position.x = P.x();
		odometry.pose.pose.position.y = P.y();
		odometry.pose.pose.position.z = P.z();
		odometry.pose.pose.orientation.x = Q.x();
		odometry.pose.pose.orientation.y = Q.y();
		odometry.pose.pose.orientation.z = Q.z();
		odometry.pose.pose.orientation.w = Q.w();

		geometry_msgs::PoseStamped pose_stamped;
		pose_stamped.header = odometry.header;
		pose_stamped.pose = odometry.pose.pose;
		refine_path.header = odometry.header;
		refine_path.poses.emplace_back(pose_stamped);
	}
	console::print_info("update visualization end.\n");
}

void KeyFrameDatabase::addLoop(int loop_index)
{
	std::unique_lock<std::mutex> lock(mPosegraphVisualization);
	if (earliest_loop_index > loop_index || earliest_loop_index == -1)
		earliest_loop_index = loop_index;

	KeyFrame* cur_KF = getLastKeyframe();
	KeyFrame* connected_KF = getKeyframe(loop_index);
	Eigen::Vector3d conncected_P, P;
	Eigen::Matrix3d connected_R, R;
	cur_KF->getPose(P, R);
	connected_KF->getPose(conncected_P, connected_R);
	//posegraph_visualization->add_loopedge(P, conncected_P);
}

nav_msgs::Path KeyFrameDatabase::getPath()
{
	std::unique_lock<std::mutex> lock(mPath);
	return refine_path;
}

//CameraPoseVisualization* KeyFrameDatabase::getPosegraphVisualization()
//{
//	unique_lock<mutex> lock(mPosegraphVisualization);
//	return posegraph_visualization;
//}
