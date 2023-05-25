#pragma once

#include "vertex.h"

namespace backend {
	class VertexPointXYZ : public Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexPointXYZ() : Vertex(3) {}

		virtual std::string TypeInfo() const 
		{ 
			return "VertexPointXYZ";
		}
	};
}
