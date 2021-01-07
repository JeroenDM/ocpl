#pragma once
#include "ocpl_planning/settings.h"

#include <ocpl_tsr/task_space_regions.h>
#include <ocpl_sampling/sampler.h>

namespace ocpl
{
SamplerPtr createSampler(const std::vector<Bounds> bounds, SamplerType type, const std::vector<int>& num_samples);

SamplerPtr createGridSampler(const std::vector<Bounds> bounds, const std::vector<int>& num_samples);

SamplerPtr createIncrementalSampler(const std::vector<Bounds> bounds, SamplerType type);
}  // namespace ocpl
