#pragma once

#include <random>
#include <pcg_random.hpp>

class JimGaussianSampler {
public:
  JimGaussianSampler(double mean, double sigma);

  double sample(void);

private:
  std::normal_distribution<double> normal_dist_;
  pcg32 rng_;
};

