#pragma once

#include "vertex.h"

namespace backend {
	class VertexPose : public Vertex {
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		VertexPose() : Vertex(7, 6) {}

		virtual void Plus(const VecXd& delta) override
		{
			auto& parameters = Parameters();
			parameters.head<3>() += delta.head<3>();
			Qd q(parameters[6], parameters[3], parameters[4], parameters[5]);
			//q = q * Sophus::SO3d::exp(Vec3d(delta[3], delta[4], delta[5])).unit_quaternion();
			parameters[3] = q.x();
			parameters[4] = q.y();
			parameters[5] = q.z();
			parameters[6] = q.w();
		}

		virtual std::string TypeInfo() const
		{
			return "VertexPose";
		}
	};
}