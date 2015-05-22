#pragma once

#include <iostream>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-register"
#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#pragma clang diagnostic pop

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor> Mat;
using Eigen::Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

//===========================================================================
class VertexPositionVelocity3D : public g2o::BaseVertex<6, Vector6d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPositionVelocity3D() {
      _estimate.setZero();
    }

  virtual void setToOriginImpl() {
    _estimate.setZero();
  }

  virtual void oplusImpl(const double* update) {
    for (int k = 0; k < 6; k++)
      _estimate[k] += update[k];
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }
};


