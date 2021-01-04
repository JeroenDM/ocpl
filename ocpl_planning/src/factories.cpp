#include <ocpl_planning/factories.h>

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>
#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>

namespace ocpl
{
SamplerPtr createGridSampler(const TSR& tsr, const std::vector<int>& num_samples)
{
    SamplerPtr sampler = std::make_shared<GridSampler>();
    sampler->addDimension(tsr.bounds.x.lower, tsr.bounds.x.upper, num_samples[0]);
    sampler->addDimension(tsr.bounds.y.lower, tsr.bounds.y.upper, num_samples[1]);
    sampler->addDimension(tsr.bounds.z.lower, tsr.bounds.z.upper, num_samples[2]);
    sampler->addDimension(tsr.bounds.rx.lower, tsr.bounds.rx.upper, num_samples[3]);
    sampler->addDimension(tsr.bounds.ry.lower, tsr.bounds.ry.upper, num_samples[4]);
    sampler->addDimension(tsr.bounds.rz.lower, tsr.bounds.rz.upper, num_samples[5]);
    return sampler;
}

SamplerPtr createIncrementalSampler(const TSR& tsr, SamplerType type)
{
    SamplerPtr sampler;
    switch (type)
    {
        case SamplerType::RANDOM: {
            sampler = std::make_shared<RandomSampler>();
            break;
        }
        case SamplerType::HALTON: {
            sampler = std::make_shared<HaltonSampler>();
            break;
        }
        default: {
            assert(false && "Jeroen, you forgot to implement a SamplerType!");
        }
    }

    sampler->addDimension(tsr.bounds.x.lower, tsr.bounds.x.upper);
    sampler->addDimension(tsr.bounds.y.lower, tsr.bounds.y.upper);
    sampler->addDimension(tsr.bounds.z.lower, tsr.bounds.z.upper);
    sampler->addDimension(tsr.bounds.rx.lower, tsr.bounds.rx.upper);
    sampler->addDimension(tsr.bounds.ry.lower, tsr.bounds.ry.upper);
    sampler->addDimension(tsr.bounds.rz.lower, tsr.bounds.rz.upper);
    return sampler;
}

}  // namespace ocpl
