#pragma once

#include <pcl/recognition/cg/geometric_consistency.h>

#include "../transformationestimation.hpp"
#include "../parameter.h"
#include "../logger.h"

namespace PoseEstimation
{
    /**
     * @brief Transformation estimation using Geometric Consistency (Correspondence Grouping).
     */
    template<typename PointT, typename DescriptorT>
    class GeometricConsistency : public TransformationEstimator<PointT, DescriptorT>
    {
    public:
        GeometricConsistency() : TransformationEstimator<PointT, DescriptorT>()
        {
            argumentCategory.define();
        }

        virtual bool estimate(PC<PointT> &source,
                              PC<PointT> &,
                              const typename pcl::PointCloud<PointT>::Ptr &source_keypoints,
                              const typename pcl::PointCloud<PointT>::Ptr &target_keypoints,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &,
                              const pcl::CorrespondencesPtr &correspondences,
                              std::vector<Eigen::Matrix4f> &transformations)
        {
            _cg.setGCSize(source.resolution() * resolution.value<float>());
            _cg.setGCThreshold(threshold.value<float>());

            _cg.setInputCloud(source_keypoints);
            _cg.setSceneCloud(target_keypoints);
            _cg.setModelSceneCorrespondences(correspondences);

            std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > trans;
            std::vector<pcl::Correspondences> clustered_corrs;
            _cg.recognize(trans, clustered_corrs);
            for (size_t i = 0; i < trans.size(); ++i)
            {
                transformations.push_back(trans[i]);
            }

            return !trans.empty();
        }

        static ParameterCategory argumentCategory;

        static Parameter resolution;
        static Parameter threshold;

    private:
        pcl::GeometricConsistencyGrouping<PointT, PointT> _cg;
    };

    template<typename PointT, typename DescriptorT>
    ParameterCategory GeometricConsistency<PointT, DescriptorT>::argumentCategory(
            "cg", "Transformation estimation using Geometric Consistency (Correspondence Grouping)",
            PipelineModuleType::TransformationEstimator);

    template<typename PointT, typename DescriptorT>
    Parameter GeometricConsistency<PointT, DescriptorT>::resolution = Parameter(
            "cg",
            "resolution",
            5.0f,
            "Consensus set resolution");

    template<typename PointT, typename DescriptorT>
    Parameter GeometricConsistency<PointT, DescriptorT>::threshold = Parameter(
            "cg",
            "thresh",
            5.0f,
            "Minimum cluster size");
}
