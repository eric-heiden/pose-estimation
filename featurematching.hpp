#ifndef FeatureMatching_H
#define FeatureMatching_H

#include <pcl/correspondence.h>

#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    template<typename DescriptorT>
    class FeatureMatcher
    {
    public:
        /**
         * @brief Finds correspondences between descriptors of the source point cloud
         * and the target.
         * @param source_descriptors Feature descriptors from the source point cloud.
         * @param target_descriptors Feature descriptors from the target point cloud.
         * @param correspondences The correspondences. Each correspondence contains
         * the index of the source (query) descriptor and the corresponding target
         * descriptor.
         */
        virtual void match(const typename pcl::PointCloud<DescriptorT>::Ptr &source_descriptors,
                           const typename pcl::PointCloud<DescriptorT>::Ptr &target_descriptors,
                           pcl::CorrespondencesPtr &correspondences) = 0;
    };
}

#endif // FeatureMatching_H
