#pragma once

#include <stdint.h>
#include <cstdlib>

#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-register"
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/sampler.h>
#pragma clang diagnostic pop

#include <pcg_random.hpp>
#include <cpp_mpl.hpp>

typedef Eigen::Matrix<cppmpl::NumpyArray::dtype, Eigen::Dynamic, Eigen::Dynamic,
        Eigen::RowMajor> Mat;
using Eigen::Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

//===========================================================================
class VertexPosition3D : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexPosition3D() {
      _estimate.setZero();
    }

  virtual void setToOriginImpl() {
    _estimate.setZero();
  }

  virtual void oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }

  virtual bool read(std::istream& /*is*/) { return false; }
  virtual bool write(std::ostream& /*os*/) const { return false; }
};

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


