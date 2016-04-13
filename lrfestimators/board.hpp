#ifndef BOARDLocalReferenceFrameEstimator_H
#define BOARDLocalReferenceFrameEstimator_H

#include <pcl/features/board.h>

#include "../lrfestimation.hpp"
#include "../types.h"
#include "../consoleargument.h"
#include "../pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Local Reference Frame Estimation using BOrder Aware Repeatable Directions (BOARD).
     */
    template<typename PointT>
    class BOARDLocalReferenceFrameEstimator : public LocalReferenceFrameEstimator<PointT>
    {
    public:
        virtual void estimate(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              pcl::PointCloud<RFType>::Ptr &lrf)
        {
            pcl::BOARDLocalReferenceFrameEstimation<PointT, NormalType, RFType> estimator;
            estimator.setFindHoles(findHoles.value<bool>());
            estimator.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            estimator.setInputCloud(keypoints);
            estimator.setInputNormals(pc.normals());
            estimator.setSearchSurface(pc.cloud());
            estimator.compute(*lrf);
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument searchRadius;
        static ConsoleArgument findHoles;
    };

    template<typename PointT>
    ConsoleArgumentCategory BOARDLocalReferenceFrameEstimator<PointT>::argumentCategory(
            "BOARD", "Local Reference Frame Estimation using BOrder Aware Repeatable Directions (BOARD)");

    template<typename PointT>
    ConsoleArgument BOARDLocalReferenceFrameEstimator<PointT>::searchRadius = ConsoleArgument(
            "BOARD", "search_r", (float)10.0f, "Search radius of BOARD LRF estimation");

    template<typename PointT>
    ConsoleArgument BOARDLocalReferenceFrameEstimator<PointT>::findHoles = ConsoleArgument(
            "BOARD", "holes", true, "Search and account for holes in the margin of the support");
}

#endif // BOARDLocalReferenceFrameEstimator_H
