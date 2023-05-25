#pragma once

#include "vertex.h"

namespace backend {
	class VertexSpeedBias : public Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexSpeedBias() : Vertex(9) {}

		virtual std::string TypeInfo() const
		{
			return "VertexSpeedBias";
		}
	};
}