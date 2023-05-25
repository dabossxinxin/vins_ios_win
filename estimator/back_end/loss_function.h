#pragma once

#include "eigen_types.h"

namespace backend {
	class LossFunction {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		
		virtual ~LossFunction() {}

		virtual void Compute(double ee, Eigen::Vector3d& rho) const = 0;
	};

	class TrivalLoss : public LossFunction {
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		virtual void Compute(double ee, Eigen::Vector3d& rho) const override
		{
			rho[0] = ee;
			rho[1] = 1;
			rho[2] = 0;
		}
	};

	// Huber(e) = e^2				if e <= delta
	// Huber(e) = delta*(2*e-delta)	if e > delta
	class HuberLoss : public LossFunction {
	public:
		explicit HuberLoss(double delta) : m_delta(delta) {}

		virtual void Compute(double ee, Eigen::Vector3d& rho) const override;
	private:
		double m_delta;
	};

	class CauthyLoss : public LossFunction {
	public:
		explicit CauthyLoss(double delta): m_delta(delta) {}

		virtual void Compute(double ee, Eigen::Vector3d& rho) const override;
	private:
		double m_delta;
	};

	class TukeyLoss : public LossFunction
	{
	public:
		explicit TukeyLoss(double delta) : m_delta(delta) {}

		virtual void Compute(double ee, Eigen::Vector3d& rho) const override;

	private:
		double m_delta;
	};
}