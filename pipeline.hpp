#pragma once

#include <memory>
#include <map>

#include "types.h"
#include "defaults.h"
#include "logger.h"
#include "parameter.h"
#include "pointcloud.h"
#include "downsampling.hpp"
#include "keypointextraction.hpp"
#include "featurematching.hpp"
#include "visualizer.h"
#include "hypothesisverification.hpp"

/**
 * @brief Namespace for Pose Estimation
 */
namespace PoseEstimation
{
    /**
     * @brief Performance statistics of the pose estimation pipeline.
     */
    struct PipelineStats
    {
        const size_t MAX_POINTS = 640 * 480; // Kinect's resolution

        size_t sourceDownsampledPoints;
        size_t targetDownsampledPoints;

        size_t sourceKeypointsExtracted;
        size_t targetKeypointsExtracted;

        size_t correspondencesFound;
        double averageCorrespondenceDistance;

        bool transformationSuccessful;
        std::vector<Eigen::Matrix4f> transformationInstances;

        /**
         * @brief Computes the performance value of the pipeline's outcome.
         * @return The performance value (the higher the better).
         */
        double performance()
        {
            // actual transformation instances are weighted much higher
            // so that we not only get a large number of keypoints
            return 0.01 * (sourceDownsampledPoints / MAX_POINTS)
                    + 0.01 * (targetDownsampledPoints / MAX_POINTS)
                    + 0.03 * (sourceKeypointsExtracted / MAX_POINTS)
                    + 0.03 * (targetKeypointsExtracted / MAX_POINTS)
                    + 0.5 * correspondencesFound
                    + (1.0 - averageCorrespondenceDistance / 30000.0)
                    + 3.0 * transformationInstances.size();
        }

        /**
         * @brief Prints the statistics in to the logger in DEBUG mode.
         */
        void print()
        {
            #define _PRINT_PROPERTY(property) \
            Logger::debug(boost::format("%1%: %2%") % #property % property)

            _PRINT_PROPERTY(sourceDownsampledPoints);
            _PRINT_PROPERTY(targetDownsampledPoints);

            _PRINT_PROPERTY(sourceKeypointsExtracted);
            _PRINT_PROPERTY(targetKeypointsExtracted);

            _PRINT_PROPERTY(correspondencesFound);
            _PRINT_PROPERTY(averageCorrespondenceDistance);

            _PRINT_PROPERTY(transformationSuccessful);
            _PRINT_PROPERTY(transformationInstances.size());
        }
    };

    /**
     * @brief The standard Pose Estimation Pipeline.
     * @details To estimate the absolute transformation of the source point cloud to the target,
     * the following steps are executed:
     * * (optional) downsampling of both point clouds
     * * keypoint extraction
     * * feature description at keypoints
     * * matching of feature descriptors to find correspondences
     * * rigid transformation estimation using corresponding feature descriptors
     * * (optional) iterative pose refinement based on priorly estimated transformation
     */
    template<typename DescriptorT, typename PointT = PointType>
    class Pipeline
    {
    public:
        /**
         * @brief Initializes the pipeline without initializing the pipeline modules.
         */
        Pipeline()
        {
            _usedModules[PipelineModuleType::Downsampler] = true;
            _usedModules[PipelineModuleType::FeatureDescriptor] = true;
            _usedModules[PipelineModuleType::FeatureMatcher] = true;
            _usedModules[PipelineModuleType::KeypointExtractor] = true;
            _usedModules[PipelineModuleType::PoseRefiner] = true;
            _usedModules[PipelineModuleType::TransformationEstimator] = true;
            _usedModules[PipelineModuleType::HypothesisVerifier] = true;

            _hypothesisVerifier = std::make_shared<HypothesisVerifier<PointT> >();
        }

        /**
         * @brief The module for feature description.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<FeatureDescriptor<PointT, DescriptorT> >& featureDescriptor()
        {
            return _featureDescriptor;
        }

        /**
         * @brief The module for point cloud downsampling.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<Downsampler<PointT> >& downsampler()
        {
            return _downsampler;
        }

        /**
         * @brief The module for keypoint extraction.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<KeypointExtractor<PointT> >& keypointExtractor()
        {
            return _keypointExtractor;
        }

        /**
         * @brief The module for transformation estimation.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<TransformationEstimator<PointT, DescriptorT> >& transformationEstimator()
        {
            return _transformationEstimator;
        }

        /**
         * @brief The module for feature matching.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<FeatureMatcher<DescriptorT> >& featureMatcher()
        {
            return _featureMatcher;
        }

        /**
         * @brief Execute the pose estimation pipeline.
         * @param source The source point cloud.
         * @param target The target point cloud.
         * @return The {@see PipelineStats} of the pipeline run.
         */
        PipelineStats process(PC<PointT> source, PC<PointT> target) const
        {
            typedef typename pcl::PointCloud<DescriptorT> PclDescriptorCloud;

            PipelineStats stats;

            // downsample
            if (_usedModules.at(PipelineModuleType::Downsampler))
            {
                Logger::log("Downsampling...");
                _downsampler->downsample(source, source);
                _downsampler->downsample(target, target);
            }

            stats.sourceDownsampledPoints = source.size();
            stats.targetDownsampledPoints = target.size();

            // keypoint extraction
            PclPointCloud::Ptr source_keypoints(new PclPointCloud);
            PclPointCloud::Ptr target_keypoints(new PclPointCloud);
            if (_usedModules.at(PipelineModuleType::KeypointExtractor))
            {
                Logger::log("Keypoint extraction...");
                _keypointExtractor->extract(source, source_keypoints);
                _keypointExtractor->extract(target, target_keypoints);
            }
            else
            {
                source_keypoints = PclPointCloud::Ptr(source.cloud());
                target_keypoints = PclPointCloud::Ptr(target.cloud());
            }

            stats.sourceKeypointsExtracted = source_keypoints->size();
            stats.targetKeypointsExtracted = target_keypoints->size();

            Logger::log("Calculating normals at keypoints...");
            PclNormalCloud::Ptr source_keypoint_normals = source.normals(source_keypoints);
            PclNormalCloud::Ptr target_keypoint_normals = target.normals(target_keypoints);

            // visualize keypoints
            PointCloud skp(source_keypoints);
            VisualizerObject vo = Visualizer::visualize(skp, Color::BLUE);
            vo.setPointSize(5.0);

            PointCloud tkp(target_keypoints);
            vo = Visualizer::visualize(tkp, Color::BLUE);
            vo.setPointSize(5.0);

            // feature description
            Logger::log("Feature description...");
            typename PclDescriptorCloud::Ptr source_features(new PclDescriptorCloud);
            typename PclDescriptorCloud::Ptr target_features(new PclDescriptorCloud);

            _featureDescriptor->describe(source, source_keypoints, source_keypoint_normals, source_features);
            Logger::debug(boost::format("Computed %d features for source point cloud.") % source_features->size());
            _featureDescriptor->describe(target, target_keypoints, target_keypoint_normals, target_features);
            Logger::debug(boost::format("Computed %d features for target point cloud.") % target_features->size());

            // matching
            Logger::log("Feature matching...");
            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
            _featureMatcher->match(source_features, target_features, correspondences);

            for (size_t j = 0; j < std::min(correspondences->size(), (size_t)500); ++j) // limit the number of displayed corrs.
            {
                PointT &model_point = source_keypoints->at((*correspondences)[j].index_query);
                PointT &scene_point = target_keypoints->at((*correspondences)[j].index_match);

                // draw line for each pair of clustered correspondences found between the model and the scene
                Visualizer::visualize(model_point, scene_point, Color::random());
            }

            stats.averageCorrespondenceDistance = FeatureMatcher<DescriptorT>::averageDistance(correspondences);
            stats.correspondencesFound = correspondences->size();

            // transformation estimation
            Logger::log("Transformation estimation...");
            std::vector<Eigen::Matrix4f> transformations;
            bool tes = _transformationEstimator->estimate(
                        source, target,
                        source_keypoints, target_keypoints,
                        source_features, target_features,
                        correspondences, transformations);
            Logger::debug(boost::format("Transformation estimation successfull? %1%") % tes);

            stats.transformationSuccessful = tes;
            stats.transformationInstances = transformations;

            if (tes)
            {
                Logger::debug(boost::format("Clusters: %1%") % transformations.size());

                if (!transformations.empty())
                {
                    for (size_t i = 0; i < std::min(transformations.size(), (size_t)80); i++) // limit number of displayed instances
                    {
                        Logger::debug(boost::format("Instance #%1%:\n%2%") % (i+1) % transformations[i]);
                        PointCloud vpc(source);
                        vpc.transform(transformations[i]);
                        VisualizerObject vpco = Visualizer::visualize(vpc, Color::YELLOW);
                        vpco.setPointSize(2.0);
                    }
                }
            }

            // pose refinement
            //TODO implement

            // hypothesis verification
            if (_usedModules.at(PipelineModuleType::HypothesisVerifier) && !transformations.empty())
            {
                std::vector<bool> mask = _hypothesisVerifier->verify(source, target, transformations);
                size_t hvc = 0;
                for (size_t i = 0; i < transformations.size() && hvc < 5; i++) // limit number of displayed instances
                {
                    if (!mask[i])
                        continue;

                    Logger::debug(boost::format("Valid hypothesis #%1%:\n%2%") % (i+1) % transformations[i]);
                    PointCloud vpc(source);
                    vpc.transform(transformations[i]);
                    VisualizerObject vpco = Visualizer::visualize(vpc, Color::GREEN);
                    vpco.setPointSize(3.0);
                    ++hvc;
                }
            }

            return stats;
        }

        /**
         * @brief Specifies whether the selected module shall be used in the pipeline.
         * @param module Type of the pipeline module.
         * @param status Whether the selected module is activated.
         */
        void useModule(PipelineModuleType::Type module, bool status)
        {
            _usedModules[module] = status;
            //TODO sanity checks, i.e. some modules depend on each other
        }

    private:
        std::shared_ptr<FeatureDescriptor<PointT, DescriptorT> > _featureDescriptor;
        std::shared_ptr<Downsampler<PointT> > _downsampler;
        std::shared_ptr<KeypointExtractor<PointT> > _keypointExtractor;
        std::shared_ptr<TransformationEstimator<PointT, DescriptorT> > _transformationEstimator;
        std::shared_ptr<FeatureMatcher<DescriptorT> > _featureMatcher;

        std::shared_ptr<HypothesisVerifier<PointT> > _hypothesisVerifier;

        std::map<PipelineModuleType::Type, bool> _usedModules;
    };
}
