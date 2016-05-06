#pragma once

#include <pcl/registration/transformation_estimation_svd.h>

#include "../transformationestimation.hpp"
#include "../parameter.h"
#include "../logger.h"

namespace PoseEstimation
{
    /**
     * @brief Transformation estimation using Singular Value Decomposition (SVD).
     */
    template<typename PointT, typename DescriptorT>
    class SVDTransformationEstimator : public TransformationEstimator<PointT, DescriptorT>
    {
    public:
        SVDTransformationEstimator() : _transformation_estimation(_SVDEstimator(true)) {} // use Umeyama
        virtual bool estimate(PC<PointT> &,
                              PC<PointT> &,
                              const typename pcl::PointCloud<PointT>::Ptr &source_keypoints,
                              const typename pcl::PointCloud<PointT>::Ptr &target_keypoints,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &,
                              const pcl::CorrespondencesPtr &correspondences,
                              std::vector<Eigen::Matrix4f> &transformations)
        {
            Eigen::Matrix4f transformation;
            _transformation_estimation.estimateRigidTransformation(
                        *source_keypoints, *target_keypoints, *correspondences, transformation);
            Logger::debug(boost::format("SVD transformation estimates:\n%1%") % transformation);
            transformations.push_back(transformation);
            return true;
        }

    private:
        typedef pcl::registration::TransformationEstimationSVD<PointT, PointType, float> _SVDEstimator;
        _SVDEstimator _transformation_estimation;
    };
}
