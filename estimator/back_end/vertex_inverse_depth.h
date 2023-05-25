#pragma once

#include "vertex.h"

namespace backend {
	
	class VertexInverseDepth : public Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexInverseDepth() : Vertex(1) {}

		virtual std::string TypeInfo() const 
		{ 
			return "VertexInverseDepth"; 
		}
	};
}