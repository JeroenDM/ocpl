#pragma once

#include <vector>

#include "ocpl_sampling/sampler.h"

namespace ocpl
{
typedef unsigned int uint;

class GridSampler : public Sampler
{
    std::vector<int> num_samples_;

    void recursiveGridSampling(int index, std::vector<double> prev_values, std::vector<std::vector<double>>& grid);

  public:
    GridSampler() = default;
    ~GridSampler() = default;
    void addDimension(double lower_bound, double upper_bound) override;
    void addDimension(double lower_bound, double upper_bound, int num_samples) override;
    std::vector<std::vector<double>> getSamples(const int n = 1) override;

    static std::vector<double> range(double lower_bound, double upper_bound, int num_samples);
};
}  // namespace ocpl
