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
#include "poserefinement.hpp"

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
        static const size_t MAX_POINTS = 640 * 480; // Kinect's resolution

        size_t sourceDownsampledPoints;
        size_t targetDownsampledPoints;

        size_t sourceKeypointsExtracted;
        size_t targetKeypointsExtracted;

        size_t correspondencesFound;
        double averageCorrespondenceDistance;
        double correspondenceSlopeVariance;

        bool transformationSuccessful;
        std::vector<Eigen::Matrix4f> transformationInstances;

        PipelineStats()
            : sourceDownsampledPoints(0),
              targetDownsampledPoints(0),
              sourceKeypointsExtracted(0),
              targetKeypointsExtracted(0),
              correspondencesFound(0),
              averageCorrespondenceDistance(std::numeric_limits<double>::max()),
              correspondenceSlopeVariance(std::numeric_limits<double>::max()),
              transformationSuccessful(false)
        {}

        /**
         * @brief Logs the statistics in DEBUG mode.
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
            _PRINT_PROPERTY(correspondenceSlopeVariance);

            _PRINT_PROPERTY(transformationSuccessful);
            _PRINT_PROPERTY(transformationInstances.size());
        }

        PipelineStats(const PipelineStats &) = default;
        PipelineStats(PipelineStats &&) = default;

        PipelineStats& operator=(const PipelineStats&) & = default;
        PipelineStats& operator=(PipelineStats&&) & = default;
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
    class Pipeline : public PipelineModule
    {
    public:
        /**
         * @brief Initializes the pipeline without initializing the pipeline modules.
         */
        Pipeline() : PipelineModule(PipelineModuleType::Miscellaneous)
        {
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
         * @brief The module for keypoint extraction.
         * @return Reference that allows for replacing the module.
         */
        std::shared_ptr<PoseRefiner<PointT> >& poseRefiner()
        {
            return _poseRefiner;
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
            if (performDownsampling.value<bool>())
            {
                Logger::log(boost::format("%s...")
                            % _downsampler->parameterCategory().description());
                _downsampler->downsample(source, source);
                _downsampler->downsample(target, target);
            }

            stats.sourceDownsampledPoints = source.size();
            stats.targetDownsampledPoints = target.size();

            // keypoint extraction
            PclPointCloud::Ptr source_keypoints(new PclPointCloud);
            PclPointCloud::Ptr target_keypoints(new PclPointCloud);
            if (performKeypointExtraction.value<bool>())
            {
                Logger::log(boost::format("%s...")
                            % _keypointExtractor->parameterCategory().description());
                _keypointExtractor->extract(source, source_keypoints);
                _keypointExtractor->extract(target, target_keypoints);

                if (source_keypoints->size() > maxDescriptors.value<int>()
                        || target_keypoints->size() > maxDescriptors.value<int>())
                {
                    Logger::warning(boost::format("Stopping pipeline, maximum number of descriptors " \
                                                  "(pipeline_max_descs = %i) was reached.")
                                % maxDescriptors.value<int>());
                    return stats;
                }
            }
            else
            {
                source_keypoints = PclPointCloud::Ptr(source.cloud());
                target_keypoints = PclPointCloud::Ptr(target.cloud());
                Logger::log("Skipping keypoint extraction.");
            }

            stats.sourceKeypointsExtracted = source_keypoints->size();
            stats.targetKeypointsExtracted = target_keypoints->size();

            Logger::log("Calculating normals at keypoints...");
            PclNormalCloud::Ptr source_keypoint_normals = source.normals(source_keypoints);
            PclNormalCloud::Ptr target_keypoint_normals = target.normals(target_keypoints);

            if (source_keypoint_normals->empty() || target_keypoint_normals->empty())
            {
                Logger::warning("Normal clouds are empty. Stopping pipeline.");
                return stats;
            }

            // visualize keypoints
            PC<PointT> skp(source_keypoints);
            VisualizerObject vo = Visualizer::visualize(skp, Color::BLUE);
            vo.setPointSize(5.0);

            PC<PointT> tkp(target_keypoints);
            vo = Visualizer::visualize(tkp, Color::BLUE);
            vo.setPointSize(5.0);

            // feature description
            Logger::log(boost::format("%s...")
                        % _featureDescriptor->parameterCategory().description());
            typename PclDescriptorCloud::Ptr source_features(new PclDescriptorCloud);
            typename PclDescriptorCloud::Ptr target_features(new PclDescriptorCloud);

            _featureDescriptor->describe(source, source_keypoints, source_keypoint_normals, source_features);
            Logger::debug(boost::format("Computed %d features for source point cloud.") % source_features->size());
            _featureDescriptor->describe(target, target_keypoints, target_keypoint_normals, target_features);
            Logger::debug(boost::format("Computed %d features for target point cloud.") % target_features->size());

            // matching
            Logger::log(boost::format("%s...")
                        % _featureMatcher->parameterCategory().description());
            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
            _featureMatcher->match(source_features, target_features, correspondences);

            const size_t MAX_DISPLAYED_CORRS = 300;
            if (correspondences->size() > MAX_DISPLAYED_CORRS)
            {
                Logger::warning(boost::format("Displaying only %i out of %i correspondences.")
                                % MAX_DISPLAYED_CORRS % correspondences->size());
            }

            for (size_t j = 0; j < std::min(correspondences->size(), MAX_DISPLAYED_CORRS); ++j) // limit the number of displayed corrs.
            {
                PointT &model_point = source_keypoints->at((*correspondences)[j].index_query);
                PointT &scene_point = target_keypoints->at((*correspondences)[j].index_match);
                float weight = (*correspondences)[j].weight;

                // draw line for each pair of clustered correspondences found between the model and the scene
                Visualizer::visualize(model_point, scene_point, Color::fromHSL(weight));
            }

            stats.averageCorrespondenceDistance = FeatureMatcher<DescriptorT>::averageDistance(correspondences);
            stats.correspondencesFound = correspondences->size();

            // compute variance of correspondence vectors
            Eigen::Vector3d *correspondenceVectors = new Eigen::Vector3d[correspondences->size()];
            Eigen::Vector3d averageCorrespondenceVector;
            for (size_t j = 0; j < correspondences->size(); ++j)
            {
                PointT &model_point = source_keypoints->at((*correspondences)[j].index_query);
                PointT &scene_point = target_keypoints->at((*correspondences)[j].index_match);

                correspondenceVectors[j] = Eigen::Vector3d(
                            scene_point.x - model_point.x,
                            scene_point.y - model_point.y,
                            scene_point.z - model_point.z).normalized();
                averageCorrespondenceVector += correspondenceVectors[j];
            }
            averageCorrespondenceVector /= (double)correspondences->size();
            Eigen::Vector3d correspondenceVectorVariance;
            for (size_t j = 0; j < correspondences->size(); ++j)
            {
                correspondenceVectorVariance += (correspondenceVectors[j] - averageCorrespondenceVector).cwiseAbs2();
            }
            delete correspondenceVectors;
            stats.correspondenceSlopeVariance = correspondenceVectorVariance.norm();

            if (skipTransformationEstimation.value<bool>())
            {
                Logger::warning("Skipping transformation estimation.");
                return stats;
            }

            // transformation estimation
            Logger::log(boost::format("%s...")
                        % _transformationEstimator->parameterCategory().description());
            std::vector<Eigen::Matrix4f> transformations;
            bool tes = _transformationEstimator->estimate(
                        source, target,
                        source_keypoints, target_keypoints,
                        source_features, target_features,
                        correspondences, transformations);
            Logger::debug(boost::format("Transformation estimation successfull? %1%") % tes);

            stats.transformationSuccessful = tes;
            stats.transformationInstances = transformations;
            //_removeDuplicates(transformations);

            if (tes)
            {                
                Logger::debug(boost::format("Clusters: %1%") % transformations.size());

                if (!transformations.empty())
                {
                    const size_t MAX_DISPLAYED_INSTANCES = 20;
                    if (transformations.size() > MAX_DISPLAYED_INSTANCES)
                    {
                        Logger::warning(boost::format("Displaying only %i out of %i instances.")
                                        % MAX_DISPLAYED_INSTANCES % transformations.size());
                    }

                    for (size_t i = 0; i < std::min(transformations.size(), MAX_DISPLAYED_INSTANCES); i++) // limit number of displayed instances
                    {
                        Logger::debug(boost::format("Instance #%1%:\n%2%") % (i+1) % transformations[i]);
                        PC<PointT> vpc(source);
                        vpc.transform(transformations[i]);
                        VisualizerObject vpco = Visualizer::visualize(vpc, Color::random(0.5));
                        vpco.setPointSize(1.0);
                        vpco.setOpacity(0.5);
                    }
                }
            }

            // pose refinement
            if (performPoseRefinement.value<bool>() && !transformations.empty())
            {
                Logger::log(boost::format("%s...")
                            % _poseRefiner->parameterCategory().description());
                for (auto &transformation : transformations)
                {
                    PC<PointT> rpc(source);
                    rpc.transform(transformation);
                    bool refined = _poseRefiner->refine(rpc, target, rpc, transformation);
                    Logger::debug(boost::format("\tRefined? %1%") % refined);
                }
            }
            else
            {
                Logger::log("Skipping pose refinement.");
            }

            // hypothesis verification
            if (performHypothesisVerification.value<bool>() && !transformations.empty())
            {
                Logger::log("Hypothesis verification...");
                std::vector<bool> mask = _hypothesisVerifier->verify(source, target, transformations);
                size_t hvc = 0;
                for (size_t i = 0; i < transformations.size() && hvc < 5; i++) // limit number of displayed instances
                {
                    if (!mask[i])
                        continue;

                    Logger::debug(boost::format("Valid hypothesis #%1%:\n%2%") % (i+1) % transformations[i]);
                    PC<PointT> vpc(source);
                    vpc.transform(transformations[i]);
                    VisualizerObject vpco = Visualizer::visualize(vpc, Color::GREEN);
                    vpco.setPointSize(3.0);
                    ++hvc;
                }
            }
            else
            {
                Logger::log("Skipping hypothesis verification.");
            }

            return stats;
        }

        __attribute__((weak))
        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        __attribute__((weak))
        static Parameter maxDescriptors;

        __attribute__((weak))
        static Parameter performDownsampling;
        __attribute__((weak))
        static Parameter performKeypointExtraction;
        __attribute__((weak))
        static Parameter performPoseRefinement;
        __attribute__((weak))
        static Parameter performHypothesisVerification;
        __attribute__((weak))
        static Parameter skipTransformationEstimation;

    private:
        std::shared_ptr<FeatureDescriptor<PointT, DescriptorT> > _featureDescriptor;
        std::shared_ptr<Downsampler<PointT> > _downsampler;
        std::shared_ptr<KeypointExtractor<PointT> > _keypointExtractor;
        std::shared_ptr<TransformationEstimator<PointT, DescriptorT> > _transformationEstimator;
        std::shared_ptr<FeatureMatcher<DescriptorT> > _featureMatcher;
        std::shared_ptr<PoseRefiner<PointT> > _poseRefiner;

        std::shared_ptr<HypothesisVerifier<PointT> > _hypothesisVerifier;

        template<typename T>
        static void _removeDuplicates(std::vector<T> &vs)
        {
            std::vector<T> seen;

            auto it = vs.begin();
            while (it != vs.end())
            {
                if (std::find(seen.begin(), seen.end(), *it) != seen.end())
                    it = vs.erase(it);
                else
                {
                    seen.push_back(*it);
                    ++it;
                }
            }
        }
    };

    template<typename DescriptorT, typename PointT>
    ParameterCategory Pipeline<DescriptorT, PointT>::argumentCategory(
            "pipeline", "Pose estimation pipeline",
            PipelineModuleType::Miscellaneous);

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::maxDescriptors = Parameter(
            "pipeline",
            "max_descs",
            300000,
            "Maximum allowable number of descriptors per cloud to be calculated");

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::performDownsampling = Parameter(
            "pipeline",
            "downsampling",
            true,
            "Whether to use the downsampling module while processing the pipeline");

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::performKeypointExtraction = Parameter(
            "pipeline",
            "keypoint",
            true,
            "Whether to use the keypoint extraction module while processing the pipeline");

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::performPoseRefinement = Parameter(
            "pipeline",
            "pose_refine",
            false, //XXX
            "Whether to use the pose refinement module while processing the pipeline");

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::performHypothesisVerification = Parameter(
            "pipeline",
            "hyp_ver",
            true,
            "Whether to use the hypothesis verification module while processing the pipeline");

    template<typename DescriptorT, typename PointT>
    Parameter Pipeline<DescriptorT, PointT>::skipTransformationEstimation = Parameter(
            "pipeline",
            "skip_te",
            false,
            "Skip transformation estimation and the following steps entirely, and only find feature correspondences.");
}
