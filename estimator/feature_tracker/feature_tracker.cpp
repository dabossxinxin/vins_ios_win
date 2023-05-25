#include "feature_tracker.h"
#include "utility/print.h"
#include "utility/camera_factory.h"

int FeatureTracker::n_id = 0;

bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
	int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
	return BORDER_SIZE <= img_x &&
		img_x < COL - BORDER_SIZE &&
		BORDER_SIZE <= img_y &&
		img_y < ROW - BORDER_SIZE;
}

void reduceVector(std::vector<cv::Point2f> &v, std::vector<uchar> status)
{
    int j = 0;
	for (int i = 0; i < int(v.size()); i++)
		if (status[i])
			v[j++] = v[i];
	v.resize(j);
}

void reduceVector(std::vector<int> &v, std::vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i])
            v[j++] = v[i];
    v.resize(j);
}

FeatureTracker::FeatureTracker()
{
}

void FeatureTracker::setMask()
{
	if (FISHEYE)
		mask = fisheye_mask.clone();
	else
		mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));

	std::vector<std::pair<int, std::pair<cv::Point2f, int>>> cnt_pts_id;
    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.emplace_back(std::make_pair(track_cnt[i], std::make_pair(forw_pts[i], ids[i])));

    sort(cnt_pts_id.begin(), cnt_pts_id.end(),[](
		const std::pair<int, std::pair<cv::Point2f, int>> &a,
                 const std::pair<int, std::pair<cv::Point2f, int>> &b) {
		return a.first > b.first;
	});

    forw_pts.clear();
    ids.clear();
	track_cnt.clear();

	// 均匀化当前帧的特征点，在同一区域中的特征点取被观测次数多的点
    for (auto &it : cnt_pts_id) {
        if (mask.at<uchar>(it.second.first) == 255) {
			forw_pts.emplace_back(it.second.first);
			ids.emplace_back(it.second.second);
			track_cnt.emplace_back(it.first);
			cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

void FeatureTracker::addPoints()
{
    for (auto &p : n_pts) {
		forw_pts.emplace_back(p);
		ids.emplace_back(-1);
		track_cnt.emplace_back(1);
    }
}

void FeatureTracker::readImage(const cv::Mat &_img)
{
	cv::Mat img;

    // 是否通过图像直方图提升光流跟踪的质量
	if (EQUALIZE) {
		//TicToc t_equalize;
		cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
		clahe->apply(_img, img);
		//console::print_info("INFO: feature_track eaqualize time: %.1f ms.\n", t_equalize.toc());
	}
	else
		img = _img;

    if (forw_img.empty()) {
		prev_img = cur_img = forw_img = img;
    }
    else {
		forw_img = img;
    }

	forw_pts.clear();

    // 上一帧特征点存在时，光流跟踪出当前帧特征点
    if (cur_pts.size() > 0) {
		std::vector<uchar> status;
		std::vector<float> err;

        //TicToc t_opticalflow;
		cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);
		//console::print_info("INFO: feature_track optical flow lk time: %.1f ms.\n", t_opticalflow.toc());

		for (int i = 0; i < int(forw_pts.size()); ++i) {
			if (status[i] && !inBorder(forw_pts[i])) {
				status[i] = 0;
			}
		}

        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
		reduceVector(track_cnt, status);
    }

    if (PUB_THIS_FRAME) {
        rejectWithF();

        for (auto &n : track_cnt)
            n++;

        //TicToc t_m;
        setMask();
        //console::print_info("INFO: feature_track set mask costs %.1f ms.\n", t_m.toc());

        //TicToc t_goodFeature;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size());
        if (n_max_cnt > 0) {
            if(mask.empty())
                console::print_error("ERROR: feature_track mask is empty.\n");
			if (mask.type() != CV_8UC1)
				console::print_error("ERROR: feature_track mask type wrong.\n");
            if (mask.size() != forw_img.size())
                console::print_error("ERROR: feature_track mask wrong size.\n");
			cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.1, MIN_DIST, mask);
        }
        else {
			n_pts.clear();
        }
        //console::print_info("INFO: feature_track goodFeaturesToTrack costs %.1f ms.\n",t_goodFeature.toc());

        addPoints();

        prev_img = forw_img;
        prev_pts = forw_pts;
    }
    cur_img = forw_img;
    cur_pts = forw_pts;
}

void FeatureTracker::rejectWithF()
{
    double half_col = COL * 0.5;
    double half_row = ROW * 0.5;
    if (forw_pts.size() >= 8) {
		std::vector<cv::Point2f> un_prev_pts(prev_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < prev_pts.size(); ++i) {
            Eigen::Vector3d tmp_p;
            m_camera->liftProjective(Eigen::Vector2d(prev_pts[i].x, prev_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + half_col;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + half_row;
            un_prev_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + half_col;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + half_row;
			un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

		std::vector<uchar> status;
        cv::findFundamentalMat(un_prev_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
    }
}

bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size()) {
        if (ids[i] == -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

void FeatureTracker::readIntrinsicParameter(const std::string &calib_file)
{
	console::print_highlight("Read camera parameters: %s\n", calib_file.c_str());
	m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

void FeatureTracker::showUndistortion(const std::string &name)
{
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
	std::vector<Eigen::Vector2d> distortedp, undistortedp;
	for (int i = 0; i < COL; ++i) {
		for (int j = 0; j < ROW; ++j) {
			Eigen::Vector2d a(i, j);
			Eigen::Vector3d b;
			m_camera->liftProjective(a, b);
			distortedp.emplace_back(a);
			undistortedp.emplace_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
		}
	}

	float half_col = COL * 0.5;
	float half_row = ROW * 0.5;
	cv::Mat pp(3, 1, CV_32FC1);
    for (int i = 0; i < int(undistortedp.size()); ++i) {
		pp.at<float>(2, 0) = 1.0;
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + half_col;
		pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + half_row;
     
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && 
			pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600) {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
    }
    cv::imshow(name, undistortedImg);
	cv::waitKey(0);
}

std::vector<cv::Point2f> FeatureTracker::undistortedPoints()
{
	std::vector<cv::Point2f> un_pts;
	for (unsigned int i = 0; i < cur_pts.size(); i++) {
		Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y);
		Eigen::Vector3d b;
		m_camera->liftProjective(a, b);
		un_pts.emplace_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
	}

	return un_pts;
}