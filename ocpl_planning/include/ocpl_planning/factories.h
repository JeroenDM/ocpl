#pragma once

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
SamplerPtr createGridSampler(const TSR& tsr, const std::vector<int>& num_samples);

SamplerPtr createIncrementalSampler(const TSR& tsr, SamplerType type);
}  // namespace ocpl
