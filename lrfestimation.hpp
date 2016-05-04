#ifndef LocalReferenceFrameEstimator_H
#define LocalReferenceFrameEstimator_H

#include "pipelinemodule.hpp"
#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    template<typename PointT>
    class LocalReferenceFrameEstimator
    {
    public:
        virtual void estimate(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              pcl::PointCloud<RFType>::Ptr &lrf) = 0;
    };
}

#endif // LocalReferenceFrameEstimator_H
