#ifndef UniformKeypointExtraction_H
#define UniformKeypointExtraction_H

#include <pcl/filters/uniform_sampling.h>

#include "../types.h"
#include "../consoleargument.h"
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
        virtual void extract(PC<PointT> &pc, typename pcl::PointCloud<PointT>::Ptr &keypoints)
        {
            _uniform_sampling.setInputCloud(pc.cloud());
            _uniform_sampling.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _uniform_sampling.filter(*keypoints);
            Logger::debug(boost::format("Extracted %d keypoints.") % keypoints->size());
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument searchRadius;

    private:
        pcl::UniformSampling<PointT> _uniform_sampling;
    };

    template<typename PointT>
    ConsoleArgumentCategory UniformKeypointExtractor<PointT>::argumentCategory(
                "uniform", "Keypoint extraction using Uniform Sampling");

    template<typename PointT>
    ConsoleArgument UniformKeypointExtractor<PointT>::searchRadius = ConsoleArgument(
                "uniform",
                "r",
                5.0f,
                "Search radius for the uniform keypoint extraction");
}

#endif // UniformKeypointExtraction_H
