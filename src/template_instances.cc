#include "random"
#include "set"
#include "string"
#include "vector"

#define EIGEN_NO_STATIC_ASSERT 1

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-register"
#include <Eigen/Core>
#include "/usr/local/include/g2o/core/base_binary_edge.h"
#include "/usr/local/include/g2o/core/base_unary_edge.h"
#include "/usr/local/include/g2o/core/block_solver.h"
#include "/usr/local/include/g2o/solvers/cholmod/linear_solver_cholmod.h"
#pragma clang diagnostic pop

#include "/usr/local/include/pcg_extras.hpp"
#include "/usr/local/include/pcg_random.hpp"

#include "types.hpp"

template class Eigen::Matrix<double, -1, -1, 1, -1, -1>;
template class Eigen::Matrix<double, 3, 1, 0, 3, 1>;
template class Eigen::Matrix<double, 3, 3, 0, 3, 3>;
template class Eigen::Matrix<double, 6, 1, 0, 6, 1>;
template class Eigen::Matrix<double, 6, 6, 0, 6, 6>;
template class g2o::BaseBinaryEdge<6, Eigen::Vector3d, class VertexPositionVelocity3D, class VertexPositionVelocity3D>;
template class g2o::BaseUnaryEdge<3, Eigen::Vector3d, class VertexPositionVelocity3D>;
template class g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >;
template class g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >::PoseMatrixType>;
template class pcg_detail::engine<unsigned int, unsigned long, struct pcg_detail::xsh_rr_mixin<unsigned int, unsigned long>, true, class pcg_detail::specific_stream<unsigned long>, struct pcg_detail::default_multiplier<uint64_t> >;
template class pcg_extras::seed_seq_from<class std::random_device>;
template class std::basic_string<char>;
template class std::normal_distribution<double>;
template class std::set<class g2o::HyperGraph::Vertex *, struct std::less<class g2o::HyperGraph::Vertex *>, class std::allocator<class g2o::HyperGraph::Vertex *> >;
