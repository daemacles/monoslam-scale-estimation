#pragma once

extern template class g2o::BaseVertex<3, Eigen::Vector3d>;
extern template class g2o::BaseVertex<6, Vector6d>;
extern template class g2o::BaseBinaryEdge<6, Eigen::Vector3d,
    VertexPositionVelocity3D, VertexPositionVelocity3D>;
extern template class g2o::BaseUnaryEdge<3, Eigen::Vector3d, VertexPositionVelocity3D>;
extern template class g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >;
extern template class g2o::LinearSolverCholmod<g2o::BlockSolver<g2o::BlockSolverTraits<6, 6> >::PoseMatrixType>;

