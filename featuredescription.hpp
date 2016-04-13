#ifndef FeatureExtraction_H
#define FeatureExtraction_H

#include <pcl/point_cloud.h>

#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    template<typename PointT, typename DescriptorT>
    class FeatureDescriptor
    {
    public:
        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              typename pcl::PointCloud<DescriptorT>::Ptr &features) = 0;
    };
}

#endif // FeatureExtraction_H
