#pragma once

#include <pcl/filters/uniform_sampling.h>

#include "../types.h"
#include "../parameter.h"
#include "../pointcloud.h"
#include "../downsampling.hpp"

namespace PoseEstimation
{
    /**
     * @brief Downsampling using uniform filtering.
     */
    template<typename PointT>
    class UniformDownsampler : public Downsampler<PointT>
    {
    public:
        UniformDownsampler() : Downsampler<PointT>()
        {
        }

        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const
        {
            if (sampleSize.value<float>() <= 1.0f)
            {
                downsampled = pc;
                return;
            }

            pcl::UniformSampling<PointT> uniform_sampling;
            uniform_sampling.setRadiusSearch(sampleSize.value<float>() * pc.resolution());
            uniform_sampling.setInputCloud(pc.cloud());
            uniform_sampling.filter(*(downsampled.cloud()));
            downsampled.update();
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter sampleSize;
    };

    template<typename PointT>
    ParameterCategory UniformDownsampler<PointT>::argumentCategory(
                "uniformdown", "Downsampling using uniform filtering",
                PipelineModuleType::Downsampler);

    template<typename PointT>
    Parameter UniformDownsampler<PointT>::sampleSize = Parameter(
                "uniformdown",
                "size",
                1.0f,
                "Sample size",
                NUMERICAL_PARAMETER_RANGE(1.0, 10.0));
}
