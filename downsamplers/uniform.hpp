#ifndef UniformDownsampling_H
#define UniformDownsampling_H

#include <pcl/filters/uniform_sampling.h>

#include "../types.h"
#include "../consoleargument.h"
#include "../pointcloud.h"
#include "../downsampling.hpp"

namespace PoseEstimation
{
    template<typename PointT>
    class UniformDownsampler : Downsampler<PointT>
    {
    public:
        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const
        {
            pcl::UniformSampling<PointT> uniform_sampling;
            uniform_sampling.setRadiusSearch(sampleSize.value<float>() * pc.resolution());
            uniform_sampling.setInputCloud(pc.cloud());
            uniform_sampling.filter(*(downsampled.cloud()));
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument sampleSize;
    };

    template<typename PointT>
    ConsoleArgumentCategory UniformDownsampler<PointT>::argumentCategory(
                "uniformdown", "Uniform downsampling");

    template<typename PointT>
    ConsoleArgument UniformDownsampler<PointT>::sampleSize = ConsoleArgument(
                "uniformdown",
                "size",
                3.0f,
                "Sample size");
}

#endif // UniformDownsampling_H
