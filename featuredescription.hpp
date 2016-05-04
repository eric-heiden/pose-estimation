#ifndef FeatureDescription_H
#define FeatureDescription_H

#include <pcl/point_cloud.h>

#include "pipelinemodule.hpp"
#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    template<typename PointT, typename DescriptorT>
    class FeatureDescriptor : public PipelineModule
    {
    public:
        FeatureDescriptor() : PipelineModule(PipelineModuleType::FeatureDescriptor)
        {}

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &normals,
                              typename pcl::PointCloud<DescriptorT>::Ptr &features) = 0;
    };
}

#endif // FeatureDescription_H
