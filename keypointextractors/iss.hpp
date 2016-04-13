#ifndef ISSKeypointExtraction_H
#define ISSKeypointExtraction_H

#include <pcl/keypoints/iss_3d.h>

#include "../types.h"
#include "../consoleargument.h"
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

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument nonMaxRadius;
        static ConsoleArgument salientRadius;
        static ConsoleArgument threshold32;
        static ConsoleArgument threshold21;
        static ConsoleArgument minNeighbors;
        static ConsoleArgument numberOfThreads;

    private:
        pcl::ISSKeypoint3D<PointT, PointT> _iss_detector;
    };

    template<typename PointT>
    ConsoleArgumentCategory ISSKeypointExtractor<PointT>::argumentCategory("iss", "Keypoint extraction using Intrinsic Shape Signatures (ISS)");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::nonMaxRadius = ConsoleArgument(
                "iss",
                "nonmax_r",
                (float)8.0f,
                "Non maxima suppression radius for ISS keypoint extraction");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::salientRadius = ConsoleArgument(
                "iss",
                "salient_r",
                (float)3.0f,
                "Salient radius for ISS keypoint extraction");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::threshold32 = ConsoleArgument(
                "iss",
                "thresh32",
                (float)0.975f,
                "ISS Threshold 32");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::threshold21 = ConsoleArgument(
                "iss",
                "thresh21",
                (float)0.975f,
                "ISS Threshold 21");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::minNeighbors = ConsoleArgument(
                "iss",
                "nn",
                (float)3.0f,
                "Minimum number of neighbors to consider for ISS keypoint extraction");

    template<typename PointT>
    ConsoleArgument ISSKeypointExtractor<PointT>::numberOfThreads = ConsoleArgument(
                "iss",
                "threads",
                (int)4,
                "Number of threads to use for ISS keypoint extraction");
}

#endif // ISSKeypointExtraction_H
