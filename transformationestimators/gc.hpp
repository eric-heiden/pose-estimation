#ifndef GC_H
#define GC_H

#include <pcl/recognition/cg/geometric_consistency.h>

#include "../transformationestimation.hpp"
#include "../consoleargument.h"
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

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument resolution;
        static ConsoleArgument threshold;
    private:
        pcl::GeometricConsistencyGrouping<PointT, PointT> _cg;
    };

    template<typename PointT, typename DescriptorT>
    ConsoleArgumentCategory GeometricConsistency<PointT, DescriptorT>::argumentCategory(
            "cg", "Transformation estimation using Geometric Consistency (Correspondence Grouping)");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument GeometricConsistency<PointT, DescriptorT>::resolution = ConsoleArgument(
            "cg",
            "resolution",
            5.0f,
            "Consensus set resolution");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument GeometricConsistency<PointT, DescriptorT>::threshold = ConsoleArgument(
            "cg",
            "thresh",
            5.0f,
            "Minimum cluster size");
}

#endif // GC_H
