#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include "utility/pinhole_camera.h"

#include <Eigen/Core>
#include <Eigen/Dense>

class CameraFactory
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	CameraFactory();

	static boost::shared_ptr<CameraFactory> instance();

	PinholeCameraPtr generateCamera(const std::string& name, cv::Size& size) const;

	PinholeCameraPtr generateCameraFromYamlFile(const std::string& filename);
private:
	static boost::shared_ptr<CameraFactory> m_instance;
};