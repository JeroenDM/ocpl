#pragma once

#include "ocpl_sampling/sampler.h"

#include <vector>
namespace ocpl
{
class HaltonSampler : public Sampler
{
    int vdc_count_{ 1 };

  public:
    HaltonSampler() = default;
    ~HaltonSampler() = default;
    void addDimension(double lower_bound, double upper_bound) override;
    std::vector<std::vector<double>> getSamples(const int n = 1) override;
    std::vector<Eigen::VectorXd> getSamplesV(const int n = 1) override;
};
}  // namespace ocpl
