#ifndef BOARDLocalReferenceFrameEstimator_H
#define BOARDLocalReferenceFrameEstimator_H

#include <pcl/features/board.h>

#include "../lrfestimation.hpp"
#include "../types.h"
#include "../parameter.h"
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

        static ParameterCategory argumentCategory;

        static Parameter searchRadius;
        static Parameter findHoles;
    };

    template<typename PointT>
    ParameterCategory BOARDLocalReferenceFrameEstimator<PointT>::argumentCategory(
            "BOARD", "Local Reference Frame Estimation using BOrder Aware Repeatable Directions (BOARD)");

    template<typename PointT>
    Parameter BOARDLocalReferenceFrameEstimator<PointT>::searchRadius = Parameter(
            "BOARD", "search_r", (float)10.0f, "Search radius of BOARD LRF estimation");

    template<typename PointT>
    Parameter BOARDLocalReferenceFrameEstimator<PointT>::findHoles = Parameter(
            "BOARD", "holes", true, "Search and account for holes in the margin of the support");
}

#endif // BOARDLocalReferenceFrameEstimator_H
