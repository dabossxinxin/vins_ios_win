#include "utility/pinhole_camera.h"

PinholeCamera::Parameters::Parameters() 
	: m_k1(0.0)
	, m_k2(0.0)
	, m_p1(0.0)
	, m_p2(0.0)
	, m_fx(0.0)
	, m_fy(0.0)
	, m_cx(0.0)
	, m_cy(0.0)
{
}

PinholeCamera::Parameters::Parameters(const std::string& cameraName,
	int width, int height,
	double k1, double k2,
	double p1, double p2,
	double fx, double fy,
	double cx, double cy)
	: m_k1(k1)
	, m_k2(k2)
	, m_p1(p1)
	, m_p2(p2)
	, m_fx(fx)
	, m_fy(fy)
	, m_cx(cx)
	, m_cy(cy)
{
}

double& PinholeCamera::Parameters::k1(void)
{
	return m_k1;
}

double& PinholeCamera::Parameters::k2(void)
{
	return m_k2;
}

double& PinholeCamera::Parameters::p1(void)
{
	return m_p1;
}

double& PinholeCamera::Parameters::p2(void)
{
	return m_p2;
}

double& PinholeCamera::Parameters::fx(void)
{
	return m_fx;
}

double& PinholeCamera::Parameters::fy(void)
{
	return m_fy;
}

double& PinholeCamera::Parameters::cx(void)
{
	return m_cx;
}

double& PinholeCamera::Parameters::cy(void)
{
	return m_cy;
}

double PinholeCamera::Parameters::k1(void) const
{
	return m_k1;
}

double PinholeCamera::Parameters::k2(void) const
{
	return m_k2;
}

double PinholeCamera::Parameters::p1(void) const
{
	return m_p1;
}

double PinholeCamera::Parameters::p2(void) const
{
	return m_p2;
}

double PinholeCamera::Parameters::fx(void) const
{
	return m_fx;
}

double PinholeCamera::Parameters::fy(void) const
{
	return m_fy;
}

double PinholeCamera::Parameters::cx(void) const
{
	return m_cx;
}

double PinholeCamera::Parameters::cy(void) const
{
	return m_cy;
}

bool PinholeCamera::Parameters::readFromYamlFile(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::READ);

	if (!fs.isOpened())
	{
		return false;
	}

	fs["camera_name"] >> m_cameraName;
	m_imageWidth = static_cast<int>(fs["image_width"]);
	m_imageHeight = static_cast<int>(fs["image_height"]);

	cv::FileNode n = fs["distortion_parameters"];
	m_k1 = static_cast<double>(n["k1"]);
	m_k2 = static_cast<double>(n["k2"]);
	m_p1 = static_cast<double>(n["p1"]);
	m_p2 = static_cast<double>(n["p2"]);

	n = fs["projection_parameters"];
	m_fx = static_cast<double>(n["fx"]);
	m_fy = static_cast<double>(n["fy"]);
	m_cx = static_cast<double>(n["cx"]);
	m_cy = static_cast<double>(n["cy"]);

	//cout << "m_k1:" << m_k1 << " m_k2" << m_k2 << " m_p1:"<<m_p1 << " m_p2:" << m_p2<< endl;
	//cout << "m_fx:" << m_fx << " m_fy" << m_fy << " m_cx:"<<m_cx << " m_cy:" << m_cy<< endl;
	return true;
}

void PinholeCamera::Parameters::writeToYamlFile(const std::string& filename)
{
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);

	fs << "model_type" << "PINHOLE";
	fs << "camera_name" << m_cameraName;
	fs << "image_width" << m_imageWidth;
	fs << "image_height" << m_imageHeight;

	// radial distortion: k1, k2
	// tangential distortion: p1, p2
	fs << "distortion_parameters";
	fs << "{" << "k1" << m_k1
		<< "k2" << m_k2
		<< "p1" << m_p1
		<< "p2" << m_p2 << "}";

	// projection: fx, fy, cx, cy
	fs << "projection_parameters";
	fs << "{" << "fx" << m_fx
		<< "fy" << m_fy
		<< "cx" << m_cx
		<< "cy" << m_cy << "}";

	fs.release();
}

PinholeCamera::Parameters::Parameters(const std::string& cameraName,
	int width, int height)
	: m_cameraName(cameraName)
	, m_imageWidth(width)
	, m_imageHeight(height)
{
	m_nIntrinsics = 8;
}

std::string& PinholeCamera::Parameters::cameraName(void)
{
	return m_cameraName;
}

int& PinholeCamera::Parameters::imageWidth(void)
{
	return m_imageWidth;
}

int& PinholeCamera::Parameters::imageHeight(void)
{
	return m_imageHeight;
}

const std::string& PinholeCamera::Parameters::cameraName(void) const
{
	return m_cameraName;
}

int PinholeCamera::Parameters::imageWidth(void) const
{
	return m_imageWidth;
}

int PinholeCamera::Parameters::imageHeight(void) const
{
	return m_imageHeight;
}

int PinholeCamera::Parameters::nIntrinsics(void) const
{
	return m_nIntrinsics;
}

const PinholeCamera::Parameters& PinholeCamera::getParameters(void) const
{
	return mParameters;
}

void PinholeCamera::setParameters(const PinholeCamera::Parameters& parameters)
{
	mParameters = parameters;

	if ((mParameters.k1() == 0.0) &&
		(mParameters.k2() == 0.0) &&
		(mParameters.p1() == 0.0) &&
		(mParameters.p2() == 0.0))
	{
		m_noDistortion = true;
	}
	else
	{
		m_noDistortion = false;
	}

	m_inv_K11 = 1.0 / mParameters.fx();
	m_inv_K13 = -mParameters.cx() / mParameters.fx();
	m_inv_K22 = 1.0 / mParameters.fy();
	m_inv_K23 = -mParameters.cy() / mParameters.fy();
}

const std::string& PinholeCamera::cameraName(void) const
{
	return mParameters.cameraName();
}

int PinholeCamera::imageWidth(void) const
{
	return mParameters.imageWidth();
}

int PinholeCamera::imageHeight(void) const
{
	return mParameters.imageHeight();
}

void PinholeCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
{
	double k1 = mParameters.k1();
	double k2 = mParameters.k2();
	double p1 = mParameters.p1();
	double p2 = mParameters.p2();

	double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

	mx2_u = p_u(0) * p_u(0);
	my2_u = p_u(1) * p_u(1);
	mxy_u = p_u(0) * p_u(1);
	rho2_u = mx2_u + my2_u;
	rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
	d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
		p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
{
	Eigen::Vector2d p_u, p_d;

	// Project points to the normalised plane
	p_u << P(0) / P(2), P(1) / P(2);

	if (m_noDistortion)
	{
		p_d = p_u;
	}
	else
	{
		// Apply distortion
		Eigen::Vector2d d_u;
		distortion(p_u, d_u);
		p_d = p_u + d_u;
	}

	// Apply generalised projection matrix
	p << mParameters.fx() * p_d(0) + mParameters.cx(),
		mParameters.fy() * p_d(1) + mParameters.cy();
}

void PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
{
	double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
	double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
	//double lambda;

	// Lift points to normalised plane
	mx_d = m_inv_K11 * p(0) + m_inv_K13;
	my_d = m_inv_K22 * p(1) + m_inv_K23;

	if (m_noDistortion)
	{
		mx_u = mx_d;
		my_u = my_d;
	}
	else
	{
		if (0)
		{
			double k1 = mParameters.k1();
			double k2 = mParameters.k2();
			double p1 = mParameters.p1();
			double p2 = mParameters.p2();

			// Apply inverse distortion model
			// proposed by Heikkila
			mx2_d = mx_d * mx_d;
			my2_d = my_d * my_d;
			mxy_d = mx_d * my_d;
			rho2_d = mx2_d + my2_d;
			rho4_d = rho2_d * rho2_d;
			radDist_d = k1 * rho2_d + k2 * rho4_d;
			Dx_d = mx_d * radDist_d + p2 * (rho2_d + 2 * mx2_d) + 2 * p1*mxy_d;
			Dy_d = my_d * radDist_d + p1 * (rho2_d + 2 * my2_d) + 2 * p2*mxy_d;
			inv_denom_d = 1 / (1 + 4 * k1*rho2_d + 6 * k2*rho4_d + 8 * p1*my_d + 8 * p2*mx_d);

			mx_u = mx_d - inv_denom_d * Dx_d;
			my_u = my_d - inv_denom_d * Dy_d;
		}
		else
		{
			// Recursive distortion model
			int n = 8;
			Eigen::Vector2d d_u;
			distortion(Eigen::Vector2d(mx_d, my_d), d_u);
			// Approximate value
			mx_u = mx_d - d_u(0);
			my_u = my_d - d_u(1);

			for (int i = 1; i < n; ++i)
			{
				distortion(Eigen::Vector2d(mx_u, my_u), d_u);
				mx_u = mx_d - d_u(0);
				my_u = my_d - d_u(1);
			}
		}
	}

	// Obtain a projective ray
	//cout<<"lift:mx_u:"<<mx_u<<",my_u:"<<my_u<<endl;
	P << mx_u, my_u, 1.0;
}