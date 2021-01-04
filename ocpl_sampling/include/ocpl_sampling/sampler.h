#pragma once

#include <vector>
#include <memory>

namespace ocpl
{
/** \class Sampler
 * \brief Abstract base class for all samplers.
 *
 * This class defines the interface that planning algorithms can use
 * to work with generic samplers, indepentent of how the samples are generated.
 * */
class Sampler
{
  protected:
    int dimensions_{ 0 };
    std::vector<double> lower_bounds_;
    std::vector<double> upper_bounds_;

    double scale(double random_value, int dim);

  public:
    virtual void addDimension(double lower_bound, double upper_bound);
    virtual void addDimension(double lower_bound, double upper_bound, int num_samples);
    virtual std::vector<std::vector<double>> getSamples(const int n = 1) = 0;
    int getNumDimensions()
    {
        return dimensions_;
    }
};

typedef std::shared_ptr<Sampler> SamplerPtr;

/** \brief Different type of incremental samplers.
 * 
 * OCPL implements different types of incremental samplers.
 * For example deterministic halton sampling.
 * **/
enum class SamplerType
{
    RANDOM,
    HALTON
};

}  // namespace ocpl
