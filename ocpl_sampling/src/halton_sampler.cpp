/** van der Corput and Halton sampling
 * based on
 * https://en.wikipedia.org/wiki/Van_der_Corput_sequence
 * */

#include "ocpl_sampling/halton_sampler.h"

#include <array>
#include <vector>
#include <stdexcept>

namespace ocpl
{
static const std::size_t NUM_PRIMES{ 59 };
static const std::array<int, NUM_PRIMES> PRIMES{ 3,   5,   7,   11,  13,  17,  19,  23,  29,  31,  37,  41,
                                                 43,  47,  53,  59,  61,  67,  71,  73,  79,  83,  89,  97,
                                                 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157,
                                                 163, 167, 173, 179, 181, 191, 193, 197, 199, 211, 223, 227,
                                                 229, 233, 239, 241, 251, 257, 263, 269, 271, 277, 281 };

double vdc(int n, int base)
{
    double q{};
    double bk{ 1.0 / base };

    while (n > 0)
    {
        q += (n % base) * bk;
        n /= base;
        bk /= base;
    }
    return q;
}

void HaltonSampler::addDimension(double lower_bound, double upper_bound)
{
    Sampler::addDimension(lower_bound, upper_bound);
    if (dimensions_ > NUM_PRIMES)
        throw std::range_error("Maximum number of dimensions Halton sampler reached.");
}

std::vector<std::vector<double>> HaltonSampler::getSamples(const int n)
{
    std::vector<std::vector<double>> samples(n);
    for (int i = 0; i < n; ++i)
    {
        for (int dim = 0; dim < dimensions_; ++dim)
        {
            if (has_tolerance_[dim])
            {
                double u = vdc(vdc_count_, PRIMES[dim]);
                samples[i].push_back(scale(u, dim));
                vdc_count_++;
            }
            else
            {
                samples[i].push_back(lower_bounds_[dim]);  // lower_bound == upper_bound
            }
        }
    }

    return samples;
}

std::vector<Eigen::VectorXd> HaltonSampler::getSamplesV(const int n)
{
    std::vector<Eigen::VectorXd> samples((size_t)n);
    for (int i = 0; i < n; ++i)
    {
        samples[i].resize(dimensions_);
        for (int dim = 0; dim < dimensions_; ++dim)
        {
            if (has_tolerance_[dim])
            {
                double u = vdc(vdc_count_, PRIMES[dim]);
                samples[i][dim] = scale(u, dim);
                vdc_count_++;
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
