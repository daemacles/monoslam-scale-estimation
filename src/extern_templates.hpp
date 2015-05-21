#pragma once

extern template class Eigen::Matrix<double, -1, -1, 1, -1, -1>;
extern template class Eigen::Matrix<double, 3, 1, 0, 3, 1>;
extern template class Eigen::Matrix<double, 3, 3, 0, 3, 3>;
extern template class Eigen::Matrix<double, 6, 1, 0, 6, 1>;
extern template class Eigen::Matrix<double, 6, 6, 0, 6, 6>;
extern template class g2o::BaseBinaryEdge<6, Eigen::Vector3d, class VertexPositionVelocity3D, class VertexPositionVelocity3D>;
extern template class g2o::BaseUnaryEdge<3, Eigen::Vector3d, class VertexPositionVelocity3D>;
extern template class g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >;
extern template class g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >::PoseMatrixType>;
extern template class pcg_detail::engine<unsigned int, unsigned long, struct pcg_detail::xsh_rr_mixin<unsigned int, unsigned long>, true, class pcg_detail::specific_stream<unsigned long>, struct pcg_detail::default_multiplier<uint64_t> >;
extern template class pcg_extras::seed_seq_from<class std::random_device>;
extern template class std::basic_string<char>;
extern template class std::normal_distribution<double>;
extern template class std::set<class g2o::HyperGraph::Vertex *, struct std::less<class g2o::HyperGraph::Vertex *>, class std::allocator<class g2o::HyperGraph::Vertex *> >;
