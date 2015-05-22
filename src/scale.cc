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

#include <cpp_mpl.hpp>

#include "sampler.hpp"

using cppmpl::NumpyArray;

cppmpl::CppMatplotlib MplConnect (void);

// Need to define types for
// - vertices in the graph
// - edges that link the vertex types

// Example for a vertex.  UNDERLYING_TYPE is the type of the _estimate field.
// class VertexType : public g2o::BaseVertex<DIMENSION, UNDERLYING_TYPE> {
// public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     VertexType() {
//       _estimate.setZero();
//     }
//
//   | kind of a "reset" method
//   virtual void setToOriginImpl() {
//     _estimate.setZero();
//   }
//
//   | has oPlusImpl that describes how a step in the minimal representation
//   | updates the state represented by this vertex
//   virtual void oplusImpl(const double* update) {
//     _estimate = ApplyUpdate(update);
//   }
//
//   | (optional) serialization
//   virtual bool read(std::istream& /*is*/) { return false; }
//   virtual bool write(std::ostream& /*os*/) const { return false; }
// };

int main (int argc, char **argv) {
  (void) argc;
  (void) argv;


  return 0;
}

cppmpl::CppMatplotlib MplConnect (void) {
  auto config_path = std::getenv("IPYTHON_KERNEL");
  if (config_path == nullptr) {
    std::cerr << "Please export IPYTHON_KERNEL=/path/to/kernel-NNN.json"
      << std::endl;
    std::exit(-1);
  }

  cppmpl::CppMatplotlib mpl{config_path};
  mpl.Connect();

  return mpl;
}
