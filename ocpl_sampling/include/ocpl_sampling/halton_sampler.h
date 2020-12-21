#pragma once

#include "ocpl_sampling/sampler.h"

#include <vector>
namespace ocpl
{
class HaltonSampler : public Sampler
{
    std::vector<int> primes_;
    int vdc_count_{ 1 };

  public:
    HaltonSampler() = default;
    ~HaltonSampler() = default;
    void addDimension(double lower_bound, double upper_bound) override;
    std::vector<std::vector<double>> getSamples(const int n = 1) override;
};
}  // namespace ocpl
