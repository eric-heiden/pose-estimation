#ifndef KeypointExtraction_H
#define KeypointExtraction_H

#include <pcl/point_cloud.h>

#include "pipelinemodule.hpp"
#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for keypoint extraction from point clouds.
     */
    template<typename PointT>
    class KeypointExtractor : public PipelineModule
    {
    public:
        KeypointExtractor() : PipelineModule(PipelineModuleType::KeypointExtractor)
        {}

        virtual void extract(PC<PointT> &pc, typename pcl::PointCloud<PointT>::Ptr &keypoints) = 0;
    };
}

#endif // KeypointExtraction_H
