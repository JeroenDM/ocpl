#include "ocpl_sampling/sampler.h"

#include <stdexcept>

namespace ocpl
{
void Sampler::addDimension(double lower_bound, double upper_bound)
{
  if (lower_bound > upper_bound)
    throw std::invalid_argument("Lower bound should be lower than upper bound.");
  dimensions_ += 1;
  lower_bounds_.push_back(lower_bound);
  upper_bounds_.push_back(upper_bound);
  has_tolerance_.push_back(lower_bound != upper_bound);
}

/** n argument not used, but I'm having a C++ design problem I cannot solve yet.
 * */
void Sampler::addDimension(double lower_bound, double upper_bound, int num_samples)
{
  addDimension(lower_bound, upper_bound);
}

double Sampler::scale(double random_value, int dim)
{
  return (upper_bounds_[dim] - lower_bounds_[dim]) * random_value + lower_bounds_[dim];
}

std::vector<double> Sampler::getSample()
{
  return getSamples(1)[0];
}
}  // namespace ocpl
