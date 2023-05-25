#include <iostream>
#include "edge.h"
#include "vertex.h"

namespace backend {
	unsigned long global_edge_id = 0;

	Edge::Edge(int residual_dimension, int verticies_num,
		const std::vector<std::string>& types) 
	{
		m_residual.resize(residual_dimension, 1);
		if (!m_verticiesTypes.empty()) {
			m_verticiesTypes = types;
		}

		m_edgeId = global_edge_id++;
		MatXXd info(residual_dimension, residual_dimension);
		info.setIdentity();
		m_information = info;
		m_sqrtInformation = info;
		m_lossFunction = nullptr;
	}

	Edge::~Edge()
	{
	}

	double Edge::Chi2() const
	{
		return m_residual.transpose()*m_information*m_residual;
	}

	double Edge::RobustChi2() const 
	{
		double ee = this->Chi2();
		if (m_lossFunction) {
			Eigen::Vector3d rho;
			m_lossFunction->Compute(ee, rho);
			ee = rho[0];
		}
		return ee;
	}

	void Edge::RobustInfo(double& drho, MatXXd& info) const
	{
		if (m_lossFunction) {
			double ee = this->Chi2();
			Eigen::Vector3d rho;
			m_lossFunction->Compute(ee, rho);
			VecXd weight_err = m_sqrtInformation * m_residual;
			
			MatXXd robust_info(m_information.rows(), m_information.cols());
			robust_info.setIdentity();
			robust_info *= rho[1];
			
			if (rho[1] + 2 * rho[2] * ee > 0) {
				robust_info += 2 * rho[2] * weight_err*weight_err.transpose();
			}

			info = robust_info * m_information;
			drho = rho[1];
		}
		else {
			drho = 1.0;
			info = m_information;
		}
	}

	bool Edge::CheckValid()
	{
		if (!m_verticiesTypes.empty()) {
			for (int it = 0; it < static_cast<int>(m_verticies.size()); ++it) {
				if (m_verticiesTypes[it] != m_verticies[it]->TypeInfo()) {
					std::cout << "Vertex types does not match, should be "
						<< m_verticiesTypes[it] << ", but set to "
						<< m_verticies[it]->TypeInfo() << std::endl;
					return false;
				}
			}
		}

		if (m_information.rows() != m_information.cols()) {
			std::cout << "Edge information matrix dimension error" << std::endl;
			return false;
		}

		if (m_residual.rows() != m_information.rows()) {
			std::cout << "Resduals or information dimension error in Edge" << std::endl;
			return false;
		}

		if (m_residual.rows() != m_observation.rows()) {
			std::cout << "Residuals or observation dimansion error in Edge" << std::endl;
			return false;
		}

		for (int it = 0; it < static_cast<int>(m_jacobians.size()); +it) {
			if (m_jacobians[it].rows() != m_residual.rows()) {
				std::cout << "Jacobians or residual dimansion error in Edge" << std::endl;
				return false;
			}

			if (m_jacobians[it].cols() != m_verticies[it]->LocalDimension()) {
				std::cout << "Jacobians or residual dimansion error in Edge" << std::endl;
				return false;
			}
		}

		return true;
	}
}