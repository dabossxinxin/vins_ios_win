#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
#include <Eigen/Dense>
#include <assert.h>
#include "parameters.h"

class FeaturePerFrame
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FeaturePerFrame(const Eigen::Matrix<double,7,1> &_point, double td)
    {
		point.x() = _point(0);
		point.y() = _point(1);
		point.z() = _point(2);
		uv.x() = _point(3);
		uv.y() = _point(4);
		velocity.x() = _point(5);
		velocity.y() = _point(6);
		cur_td = td;
    }
	double cur_td;
	Eigen::Vector3d point;
	Eigen::Vector2d uv;
	Eigen::Vector2d velocity;
    double z;
    bool is_used;
    double parallax;
	Eigen::MatrixXd A;
	Eigen::VectorXd b;
    double dep_gradient;
};

class FeaturePerId
{
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const int feature_id;
    int start_frame;
	std::vector<FeaturePerFrame> feature_per_frame;

    int used_num;
    bool is_outlier;
    bool is_margin;
    double estimated_depth;
    int solve_flag; // 0 haven't solve yet; 1 solve succ; 2 solve fail;

	Eigen::Vector3d gt_p;

    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0)
    {
    }

    int endFrame();
};

class FeatureManager
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	FeatureManager(Eigen::Matrix3d _Rs[]);

	void setRic(Eigen::Matrix3d _ric[]);

	void clearState();

	int getFeatureCount();

	bool addFeatureCheckParallax(int frame_count, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
	void debugShow();
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

	//void updateDepth(const VectorXd &x);
	void setDepth(const Eigen::VectorXd &x);
	void removeFailures();
	void clearDepth(const Eigen::VectorXd &x);
	Eigen::VectorXd getDepthVector();
	void triangulate(Eigen::Vector3d Ps[], Eigen::Vector3d tic[], Eigen::Matrix3d ric[]);
	void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);
	void removeBack();
	void removeFront(int frame_count);
	void removeOutlier();
	std::list<FeaturePerId> feature;
	int last_track_num;

private:
	double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);
	const Eigen::Matrix3d *Rs;
	Eigen::Matrix3d ric[NUM_OF_CAM];
};
#endif