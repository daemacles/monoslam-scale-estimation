#include "sampler.hpp"

JimGaussianSampler::JimGaussianSampler(double mean, double sigma) :
    normal_dist_(mean, sigma),
    rng_(pcg_extras::seed_seq_from<std::random_device>())
{}

double JimGaussianSampler::sample(void) {
  return normal_dist_(rng_);
}


