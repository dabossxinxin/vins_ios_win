#pragma once

#include "eigen_types.h"
#include "loss_function.h"

namespace backend {

	class Vertex;

	class Edge {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit Edge(int residual_dimension, int verticies_num,
			const std::vector<std::string>& types = std::vector<std::string>());

		virtual ~Edge();

		unsigned long Id() const 
		{
			return m_edgeId;
		}

		bool AddVertex(std::shared_ptr<Vertex>& vertex) {
			m_verticies.emplace_back(vertex);
			return true;
		}

		bool SetVertex(const std::vector<std::shared_ptr<Vertex>>& vertex) 
		{
			m_verticies = vertex;
			return true;
		}

		std::shared_ptr<Vertex> GetVertex(int id) 
		{
			return m_verticies[id];
		}

		std::vector<std::shared_ptr<Vertex>> Verticies() const 
		{
			return m_verticies;
		}

		size_t VerticesNum() const 
		{
			return m_verticies.size();
		}

		virtual std::string TypeInfo() const = 0;
		
		virtual void ComputeResidual() = 0;

		virtual void ComputeJacobians() = 0;

		double Chi2() const;

		double RobustChi2() const;

		VecXd Residual() const
		{
			return m_residual;
		}

		VecMatXXd Jacobians() const 
		{
			return m_jacobians;
		}

		void SetInformatio(const MatXXd& info)
		{
			m_information = info;
			m_sqrtInformation = Eigen::LLT<MatXXd>(m_information).matrixL().transpose();
		}

		MatXXd Information() const 
		{
			return m_information;
		}

		MatXXd SqrtInformation() const
		{
			return m_sqrtInformation;
		}

		void SetLossFunction(LossFunction* loss) 
		{
			m_lossFunction = loss;
		}

		LossFunction* GetLossFunction() 
		{
			return m_lossFunction;
		}

		void RobustInfo(double& drho, MatXXd& info) const;

		void SetObservation(const VecXd& obs)
		{
			m_observation = obs;
		}

		VecXd Observation() const
		{
			return m_observation;
		}

		bool CheckValid();

		unsigned long OrderedId() const
		{
			return m_edgeOrderedId;
		}

		void SetOrderedId(unsigned long id)
		{
			m_edgeOrderedId = id;
		}

	protected:
		unsigned long	m_edgeId;
		unsigned long	m_edgeOrderedId;
		LossFunction	*m_lossFunction;

		std::vector<std::string>				m_verticiesTypes;
		std::vector<std::shared_ptr<Vertex>>	m_verticies;
		
		VecXd			m_residual;	// ²Ð²î=Ô¤²â-¹Û²â
		VecXd			m_observation;
		MatXXd			m_information;
		MatXXd			m_sqrtInformation;
		VecMatXXd		m_jacobians;
	};
}