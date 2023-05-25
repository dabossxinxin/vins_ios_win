#pragma once

#include "loss_function.h"

namespace backend {
	void HuberLoss::Compute(double ee, Eigen::Vector3d& rho) const
	{
		double dsqr = m_delta * m_delta;
		if (ee <= dsqr) { // inlier
			rho[0] = ee;
			rho[1] = 1.;
			rho[2] = 0.;
		}
		else { // outlier
			double sqrte = sqrt(ee);
			rho[0] = 2 * sqrte*m_delta - dsqr;
			rho[1] = m_delta / sqrte;
			rho[2] = -0.5 * rho[1] / ee;
		}
	}

	void CauthyLoss::Compute(double ee, Eigen::Vector3d& rho) const
	{
		double dsqr = m_delta * m_delta;
		double dsqrReci = 1. / dsqr;
		double aux = dsqrReci * ee + 1.0;
		rho[0] = dsqr * log(aux);
		rho[1] = 1. / aux;
		rho[2] = -dsqrReci * std::pow(rho[1], 2.0);
	}

	void TukeyLoss::Compute(double ee, Eigen::Vector3d& rho) const
	{
		const double e = sqrt(ee);
		const double delta2 = m_delta * m_delta;
		if (e <= m_delta) {
			const double aux = ee / delta2;
			rho[0] = delta2 * (1. - std::pow((1. - aux), 3)) / 3.;
			rho[1] = std::pow((1. - aux), 2);
			rho[2] = -2. * (1. - aux) / delta2;
		}
		else {
			rho[0] = delta2 / 3.;
			rho[1] = 0;
			rho[2] = 0;
		}
	}
}