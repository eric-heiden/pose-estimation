#pragma once

#include <vector>
#include <Eigen/Geometry>

#include "pipelinemodule.hpp"
#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for transformation estimation between two point clouds.
     */
    template<typename PointT, typename DescriptorT>
    class TransformationEstimator : public PipelineModule
    {
    public:
        TransformationEstimator() : PipelineModule(PipelineModuleType::TransformationEstimator)
        {}

        /**
         * @brief Estimates the rigid transformation of the source to the target point cloud
         * using the correspondences of descriptors calculated at the keypoints of both
         * point clouds.
         * @param source The source point cloud.
         * @param target The target point cloud.
         * @param source_keypoints The keypoints of the source point cloud at which
         * descriptors were calculated.
         * @param target_keypoints The keypoints of the target point cloud at which
         * descriptors were calculated.
         * @param source_features The descriptors of the source point cloud.
         * @param target_features The descriptors of the target point cloud.
         * @param correspondences The correspondences between source and target descriptors.
         * @param transformations The resulting 4x4 transformation matrix of the estimated
         * rigid transformation.
         * @return Whether the estimation was successfull.
         */
        // not every transformation estimator uses all of these arguments
        virtual bool estimate(PC<PointT> &source,
                              PC<PointT> &target,
                              const typename pcl::PointCloud<PointT>::Ptr &source_keypoints,
                              const typename pcl::PointCloud<PointT>::Ptr &target_keypoints,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &source_features,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &target_features,
                              const pcl::CorrespondencesPtr &correspondences,
                              std::vector<Eigen::Matrix4f> &transformations) = 0;
    };
}
