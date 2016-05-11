#pragma once

#include <pcl/filters/uniform_sampling.h>

#include "../types.h"
#include "../parameter.h"
#include "../pointcloud.h"
#include "../keypointextraction.hpp"

namespace PoseEstimation
{
    /**
     * @brief Keypoint extraction using uniform sampling.
     */
    //TODO this is the same as uniform downsampling: merge downsampling & keypoint extraction?
    template<typename PointT>
    class UniformKeypointExtractor : public KeypointExtractor<PointT>
    {
    public:
        UniformKeypointExtractor() : KeypointExtractor<PointT>()
        {
        }

        virtual void extract(PC<PointT> &pc, typename pcl::PointCloud<PointT>::Ptr &keypoints)
        {
            _uniform_sampling.setInputCloud(pc.cloud());
            _uniform_sampling.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _uniform_sampling.filter(*keypoints);
            Logger::debug(boost::format("Extracted %d keypoints.") % keypoints->size());
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter searchRadius;


    private:
        pcl::UniformSampling<PointT> _uniform_sampling;
    };

    template<typename PointT>
    ParameterCategory UniformKeypointExtractor<PointT>::argumentCategory(
                "uniform", "Keypoint extraction using Uniform Sampling",
                PipelineModuleType::KeypointExtractor);

    template<typename PointT>
    Parameter UniformKeypointExtractor<PointT>::searchRadius = Parameter(
                "uniform",
                "r",
                15.0f,
                "Search radius for the uniform keypoint extraction");
}
