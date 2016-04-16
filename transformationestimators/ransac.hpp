#ifndef RANSACTransformationEstimation_H
#define RANSACTransformationEstimation_H

#include <pcl/registration/sample_consensus_prerejective.h>

#include "../transformationestimation.hpp"
#include "../parameter.h"
#include "../logger.h"

namespace PoseEstimation
{
    /**
     * @brief Transformation estimation using prerejective RANSAC.
     */
    template<typename PointT, typename DescriptorT>
    class RANSACTransformationEstimator : public TransformationEstimator<PointT, DescriptorT>
    {
    public:
        virtual bool estimate(PC<PointT> &source,
                              PC<PointT> &,
                              const typename pcl::PointCloud<PointT>::Ptr &source_keypoints,
                              const typename pcl::PointCloud<PointT>::Ptr &target_keypoints,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &source_features,
                              const typename pcl::PointCloud<DescriptorT>::Ptr &target_features,
                              const pcl::CorrespondencesPtr &,
                              std::vector<Eigen::Matrix4f> &transformations)
        {
            _align.setInputSource(source_keypoints);
            _align.setSourceFeatures(source_features);
            _align.setInputTarget(target_keypoints);
            _align.setTargetFeatures(target_features);
            _align.setMaximumIterations(maxIterations.value<int>()); // Number of RANSAC iterations
            _align.setNumberOfSamples(samples.value<int>()); // Number of points to sample for generating/prerejecting a pose
            _align.setCorrespondenceRandomness(nearestFeatures.value<int>()); // Number of nearest features to use
            _align.setSimilarityThreshold(similarityThreshold.value<float>()); // Polygonal edge length similarity threshold
            _align.setMaxCorrespondenceDistance(correspondenceThreshold.value<float>() * source.resolution()); // Inlier threshold
            _align.setInlierFraction(inlierFraction.value<float>()); // Required inlier fraction for accepting a pose hypothesis

            _align.align(*source_keypoints);

            if (_align.hasConverged())
            {
                transformations.push_back(_align.getFinalTransformation());
            }
            else
            {
                Logger::error("RANSAC Transformation Estimation did not converge.");
            }

            return _align.hasConverged();
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument inlierFraction;
        static ConsoleArgument similarityThreshold;
        static ConsoleArgument correspondenceThreshold;
        static ConsoleArgument nearestFeatures;
        static ConsoleArgument maxIterations;
        static ConsoleArgument samples;
    private:
        pcl::SampleConsensusPrerejective<PointT, PointT, DescriptorT> _align;
    };

    template<typename PointT, typename DescriptorT>
    ConsoleArgumentCategory RANSACTransformationEstimator<PointT, DescriptorT>::argumentCategory(
                "ransac", "Transformation estimation using prerejective RANSAC");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::maxIterations = ConsoleArgument(
                "ransac",
                "iter",
                50000,
                "Maximum number of iterations");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::samples = ConsoleArgument(
                "ransac",
                "samples",
                3,
                "Number of points to sample for generating/prerejecting a pose");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::nearestFeatures = ConsoleArgument(
                "ransac",
                "features",
                5,
                "Number of nearest features to use");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::similarityThreshold = ConsoleArgument(
                "ransac",
                "sim_thresh",
                0.9f,
                "Polygonal edge length similarity threshold");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::correspondenceThreshold = ConsoleArgument(
                "ransac",
                "corr_thresh",
                2.5f,
                "Maximum correspondence distance for inlier consideration");

    template<typename PointT, typename DescriptorT>
    ConsoleArgument RANSACTransformationEstimator<PointT, DescriptorT>::inlierFraction = ConsoleArgument(
                "ransac",
                "hyp_thresh",
                0.25f,
                "Required inlier fraction for accepting a pose hypothesis");
}

#endif // RANSACTransformationEstimation_H
