#pragma once

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include "../utility/utility.h"

class PoseLocalParameterization : public ceres::Manifold
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const override;
    virtual bool PlusJacobian(const double *x, double *jacobian) const override;
    virtual bool Minus(const double *y, const double *x, double *y_minus_x) const override;
    virtual bool MinusJacobian(const double* x, double* jacobian) const override;
    virtual int AmbientSize() const override { return 7; };
    virtual int TangentSize() const override { return 6; };
};
