#pragma once

#include <pcl/keypoints/iss_3d.h>

#include "../types.h"
#include "../parameter.h"
#include "../pointcloud.h"
#include "../keypointextraction.hpp"

namespace PoseEstimation
{
    /**
     * @brief Keypoint extraction using Intrinsic Shape Signatures (ISS) without border estimation.
     */
    template<typename PointT>
    class ISSKeypointExtractor : public KeypointExtractor<PointT>
    {
    public:
        ISSKeypointExtractor() : KeypointExtractor<PointT>()
        {
        }

        virtual void extract(PC<PointT> &pc, typename pcl::PointCloud<PointT>::Ptr &keypoints)
        {
            _iss_detector.setNumberOfThreads(numberOfThreads.value<int>());
            _iss_detector.setMinNeighbors(minNeighbors.value<float>());
            _iss_detector.setThreshold21(threshold21.value<float>());
            _iss_detector.setThreshold32(threshold32.value<float>());
            _iss_detector.setSalientRadius(pc.resolution() * salientRadius.value<float>());
            _iss_detector.setNonMaxRadius(pc.resolution() * nonMaxRadius.value<float>());
            _iss_detector.setInputCloud(pc.cloud());

            Logger::tic("ISS Keypoint Extraction");
            _iss_detector.compute(*keypoints);
            Logger::toc("ISS Keypoint Extraction");
            Logger::debug(boost::format("Extracted %d keypoints.") % keypoints->size());
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter nonMaxRadius;
        static Parameter salientRadius;
        static Parameter threshold32;
        static Parameter threshold21;
        static Parameter minNeighbors;
        static Parameter numberOfThreads;

    private:
        pcl::ISSKeypoint3D<PointT, PointT> _iss_detector;
    };

    template<typename PointT>
    ParameterCategory ISSKeypointExtractor<PointT>::argumentCategory(
                "iss", "Keypoint extraction using Intrinsic Shape Signatures (ISS)",
                PipelineModuleType::KeypointExtractor);

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::nonMaxRadius = Parameter(
                "iss",
                "nonmax_r",
                (float)8.0f,
                "Non maxima suppression radius for ISS keypoint extraction");

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::salientRadius = Parameter(
                "iss",
                "salient_r",
                (float)3.0f,
                "Salient radius for ISS keypoint extraction");

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::threshold32 = Parameter(
                "iss",
                "thresh32",
                (float)0.975f,
                "ISS Threshold 32");

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::threshold21 = Parameter(
                "iss",
                "thresh21",
                (float)0.975f,
                "ISS Threshold 21");

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::minNeighbors = Parameter(
                "iss",
                "nn",
                (float)3.0f,
                "Minimum number of neighbors to consider for ISS keypoint extraction");

    template<typename PointT>
    Parameter ISSKeypointExtractor<PointT>::numberOfThreads = Parameter(
                "iss",
                "threads",
                (int)4,
                "Number of threads to use for ISS keypoint extraction");
}
