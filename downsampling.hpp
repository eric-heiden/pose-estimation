#pragma once

#include "pipelinemodule.hpp"
#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for downsampling point clouds.
     */
    template<typename PointT>
    class Downsampler : public PipelineModule
    {
    public:
        Downsampler() : PipelineModule(PipelineModuleType::Downsampler)
        {}

        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const = 0;
    };
}
