#include "ocpl_sampling/random_sampler.h"

namespace ocpl
{
std::vector<std::vector<double>> RandomSampler::getSamples(const int n)
{
    std::vector<std::vector<double>> samples(n);
    for (int i = 0; i < n; ++i)
    {
        for (int dim = 0; dim < dimensions_; ++dim)
        {
            if (has_tolerance_[dim])
            {
                double u = randu_(random_engine_);
                samples[i].push_back(scale(u, dim));
            }
            else
            {
                samples[i].push_back(lower_bounds_[dim]);  // lower_bound == upper_bound
            }
        }
    }

    return samples;
}

std::vector<Eigen::VectorXd> RandomSampler::getSamplesV(const int n)
{
    std::vector<Eigen::VectorXd> samples((size_t) n);
    for (int i = 0; i < n; ++i)
    {
        samples[i].resize(dimensions_);
        for (int dim = 0; dim < dimensions_; ++dim)
        {
            if (has_tolerance_[dim])
            {
                double u = randu_(random_engine_);
                samples[i][dim] = scale(u, dim);
            }
            else
            {
                samples[i][dim] = lower_bounds_[dim];  // lower_bound == upper_bound
            }
        }
    }

    return samples;
}
}  // namespace ocpl
