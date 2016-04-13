#ifndef KeypointExtraction_H
#define KeypointExtraction_H

#include <pcl/point_cloud.h>

#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for keypoint extraction from point clouds.
     */
    template<typename PointT>
    class KeypointExtractor
    {
    public:
        virtual void extract(PC<PointT> &pc, typename pcl::PointCloud<PointT>::Ptr &keypoints) = 0;
    };
}

#endif // KeypointExtraction_H
