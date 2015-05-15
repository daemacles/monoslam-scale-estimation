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

#include "types.hpp"

using namespace g2o;

//template class Eigen::Matrix<double, 6, 1>;
//template class Eigen::Matrix<double, 6, 6>;
//template class Eigen::Matrix<cppmpl::NumpyArray::dtype, Eigen::Dynamic, Eigen::Dynamic,
//        Eigen::RowMajor>;
template class g2o::BaseVertex<3, Eigen::Vector3d>;
template class g2o::BaseVertex<6, Vector6d>;
template class g2o::BaseBinaryEdge<6, Eigen::Vector3d,
    VertexPositionVelocity3D, VertexPositionVelocity3D>;
template class g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D>;
template class g2o::BlockSolver< BlockSolverTraits<6, 6> >;
template class g2o::LinearSolverCholmod<g2o::BlockSolver<BlockSolverTraits<6, 6> >::PoseMatrixType>;
