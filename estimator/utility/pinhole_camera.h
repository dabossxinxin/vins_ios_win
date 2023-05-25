#pragma once

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

class PinholeCamera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	class Parameters
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		Parameters();

		Parameters(const std::string& name, int width, int height);

		Parameters(const std::string& cameraName,
			int width, int height,
			double k1, double k2, double p1, double p2,
			double fx, double fy, double cx, double cy);

		double& k1(void);
		double& k2(void);
		double& p1(void);
		double& p2(void);
		double& fx(void);
		double& fy(void);
		double& cx(void);
		double& cy(void);

		double k1(void) const;
		double k2(void) const;
		double p1(void) const;
		double p2(void) const;
		double fx(void) const;
		double fy(void) const;
		double cx(void) const;
		double cy(void) const;

		std::string& cameraName();
		int& imageWidth();
		int& imageHeight();

		const std::string& cameraName(void) const;
		int imageWidth(void) const;
		int imageHeight(void) const;

		int nIntrinsics(void) const;

		bool readFromYamlFile(const std::string& filename);
		void writeToYamlFile(const std::string& filename);

	private:
		int m_nIntrinsics;
		std::string m_cameraName;
		int m_imageWidth;
		int m_imageHeight;

		double m_k1;
		double m_k2;
		double m_p1;
		double m_p2;
		double m_fx;
		double m_fy;
		double m_cx;
		double m_cy;
	};

	const Parameters& getParameters(void) const;
	void setParameters(const Parameters& parameters);

	const std::string& cameraName(void) const;
	int imageWidth(void) const;
	int imageHeight(void) const;

	void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
	void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const;
	void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const;

private:
	Parameters mParameters;

	double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
	bool m_noDistortion;
};

typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;