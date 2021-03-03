#include <ocpl_planning/factories.h>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>

// #include <iostream>

namespace ocpl
{
SamplerPtr createSampler(const std::vector<Bounds> bounds, SamplerType type, const std::vector<int>& num_samples)
{
    SamplerPtr sampler{ nullptr };
    if (type == SamplerType::GRID)
    {
        sampler = createGridSampler(bounds, num_samples);  // TODO error
    }
    else
    {
        sampler = createIncrementalSampler(bounds, type);
    }
    return sampler;
}

SamplerPtr createGridSampler(const std::vector<Bounds> bounds, const std::vector<int>& num_samples)
{
    // std::cout << " Using a grid sampler! \n";
    assert(bounds.size() == num_samples.size());
    SamplerPtr sampler = std::make_shared<GridSampler>();
    for (std::size_t dim{ 0 }; dim < bounds.size(); ++dim)
    {
        sampler->addDimension(bounds[dim].lower, bounds[dim].upper, num_samples[dim]);
    }
    return sampler;
}

SamplerPtr createIncrementalSampler(const std::vector<Bounds> bounds, SamplerType type)
{
    SamplerPtr sampler;
    switch (type)
    {
        case SamplerType::RANDOM: {
            // std::cout << " Using a random sampler! \n";
            sampler = std::make_shared<RandomSampler>();
            break;
        }
        case SamplerType::HALTON: {
            // std::cout << " Using a halton sampler! \n";
            sampler = std::make_shared<HaltonSampler>();
            break;
        }
        default: {
            assert(false && "Jeroen, you forgot to implement a SamplerType!");
        }
    }

    for (std::size_t dim{ 0 }; dim < bounds.size(); ++dim)
    {
        sampler->addDimension(bounds[dim].lower, bounds[dim].upper);
    }
    return sampler;
}

ocpl::SamplerPtr createLocalSampler(size_t dimensions, double max_deviation, SamplerType type)
{
    std::vector<Bounds> bounds;
    for (size_t i{ 0 }; i < dimensions; ++i)
    {
        bounds.push_back({ -max_deviation, max_deviation });
    }
    return createIncrementalSampler(bounds, type);
}

ocpl::SamplerPtr createLocalSampler(size_t dimensions, double max_deviation, SamplerType type,
                                    const std::vector<int>& num_samples)
{
    std::vector<Bounds> bounds;
    for (size_t i{ 0 }; i < dimensions; ++i)
    {
        bounds.push_back({ -max_deviation, max_deviation });
    }
    return createSampler(bounds, type, num_samples);
}

}  // namespace ocpl
