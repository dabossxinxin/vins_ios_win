#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include "utility/camera_factory.h"
#include "utility/print.h"

#include <Eigen/Core>
#include <Eigen/Dense>

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{
}

boost::shared_ptr<CameraFactory> CameraFactory::instance(void)
{
	if (m_instance.get() == 0)
	{
		m_instance.reset(new CameraFactory);
	}

	return m_instance;
}

PinholeCameraPtr CameraFactory::generateCamera(const std::string& name, cv::Size& imageSize) const
{
	PinholeCameraPtr camera(new PinholeCamera);
	PinholeCamera::Parameters params = camera->getParameters();
	params.cameraName() = name;
	params.imageWidth() = imageSize.width;
	params.imageHeight() = imageSize.height;
	camera->setParameters(params);

	return camera;
}

PinholeCameraPtr CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened()) {
		return PinholeCameraPtr();
	}

	PinholeCameraPtr camera(new PinholeCamera);
	PinholeCamera::Parameters params = camera->getParameters();
	params.readFromYamlFile(filename);
	camera->setParameters(params);

	console::print_value("	fx: %f\n", params.fx());
	console::print_value("	fy: %f\n", params.fy());
	console::print_value("	cx: %f\n", params.cx());
	console::print_value("	cy: %f\n", params.cy());
	console::print_value("	k1: %f\n", params.k1());
	console::print_value("	k2: %f\n", params.k2());
	console::print_value("	p1: %f\n", params.p1());
	console::print_value("	p2: %f\n", params.p2());

	return camera;
}