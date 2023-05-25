#pragma once

#include "eigen_types.h"

namespace backend {
	class Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		explicit Vertex(int dimension, int local_dimension = -1);

		virtual ~Vertex();

		unsigned long Id() const
		{
			return m_vertexId;
		}

		unsigned long OrderedId() const
		{
			return m_vertexOrderedId;
		}

		void SetOrderingId(unsigned long id)
		{
			m_vertexOrderedId = id;
		}

		int Dimension() const
		{
			return m_dimension;
		}

		int LocalDimension() const
		{
			return m_localDimension;
		}

		VecXd Parameters() const 
		{
			return m_parameters;
		}

		VecXd& Paramters()
		{
			return m_parameters;
		}

		void SetParameters(const VecXd& para)
		{
			m_parameters = para;
		}

		void BackupParamters()
		{
			m_parametersBackup = m_parameters;
		}

		void RollBackParameters()
		{
			m_parameters = m_parametersBackup;
		}

		void SetFixed(bool fixed = true)
		{
			m_isFixed = fixed;
		}

		bool IsFixed() const
		{
			return m_isFixed;
		}

		virtual std::string TypeInfo() const = 0;
		
		virtual void Plus(const VecXd& delta);

	protected:
		VecXd m_parameters;
		VecXd m_parametersBackup;

		int m_dimension;
		int m_localDimension;

		unsigned long m_vertexId;
		unsigned long m_vertexOrderedId;

		bool m_isFixed = false;
	};
}