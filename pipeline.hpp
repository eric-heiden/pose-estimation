#ifndef Pipeline_H
#define Pipeline_H

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

namespace PoseEstimation
{
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
    template<typename DescriptorT>
    class Pipeline
    {
    public:
        Pipeline()
        {
            _featureDescriptor = std::make_shared<DefaultFeatureDescriptor>(DefaultFeatureDescriptor());
            _downsampler = std::make_shared<DefaultDownsampler>(DefaultDownsampler());
            _keypointExtractor = std::make_shared<DefaultKeypointExtractor>(DefaultKeypointExtractor());
            _transformationEstimator = std::make_shared<DefaultTransformationEstimator>(DefaultTransformationEstimator());
            _featureMatcher = std::make_shared<DefaultFeatureMatcher>(DefaultFeatureMatcher());
        }

        std::shared_ptr<FeatureDescriptor<PointType, DescriptorT> >& featureDescriptor()
        {
            return _featureDescriptor;
        }

        std::shared_ptr<Downsampler<PointType> >& downsampler()
        {
            return _downsampler;
        }

        std::shared_ptr<KeypointExtractor<PointType> >& keypointExtractor()
        {
            return _keypointExtractor;
        }

        std::shared_ptr<TransformationEstimator<PointType, DescriptorT> >& transformationEstimator()
        {
            return _transformationEstimator;
        }

        std::shared_ptr<FeatureMatcher<DescriptorType> >& featureMatcher()
        {
            return _featureMatcher;
        }

        void process(PointCloud &source, PointCloud &target) const
        {
            typedef typename pcl::PointCloud<DescriptorT> PclDescriptorCloud;

            // downsample
            Logger::log("Downsampling...");
            _downsampler->downsample(source, source);
            _downsampler->downsample(target, target);

            // keypoint extraction
            Logger::log("Keypoint extraction...");
            PclPointCloud::Ptr source_keypoints(new PclPointCloud);
            PclPointCloud::Ptr target_keypoints(new PclPointCloud);
            _keypointExtractor->extract(source, source_keypoints);
            _keypointExtractor->extract(target, target_keypoints);

            // visualize keypoints
            PointCloud skp(source_keypoints);
            VisualizerObject vo = Visualizer::visualize(skp, Color::BLUE);
            vo.setPointSize(5.0);

            PointCloud tkp(target_keypoints);
            vo = Visualizer::visualize(tkp, Color::BLUE);
            vo.setPointSize(5.0);

            // feature description

            PclNormalCloud::Ptr source_keypoint_normals = source.normals(source_keypoints);
            PclNormalCloud::Ptr target_keypoint_normals = target.normals(target_keypoints);

            Logger::log("Feature description...");
            typename PclDescriptorCloud::Ptr source_features(new PclDescriptorCloud);
            typename PclDescriptorCloud::Ptr target_features(new PclDescriptorCloud);
            Logger::debug("before describing...");
            _featureDescriptor->describe(source, source_keypoints, source_keypoint_normals, source_features);
            Logger::debug(boost::format("Found %d features for source point cloud.") % source_features->size());
            _featureDescriptor->describe(target, target_keypoints, target_keypoint_normals, target_features);
            Logger::debug(boost::format("Found %d features for target point cloud.") % target_features->size());

            // matching
            Logger::log("Feature matching...");
            DefaultFeatureMatcher fm;
            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
            fm.match(source_features, target_features, correspondences);

            Logger::debug("still works");

            for (size_t j = 0; j < correspondences->size(); ++j)
            {
                PointType &model_point = source_keypoints->at((*correspondences)[j].index_query);
                PointType &scene_point = target_keypoints->at((*correspondences)[j].index_match);

                // draw line for each pair of clustered correspondences found between the model and the scene
                Visualizer::visualize(model_point, scene_point, Color::random());
            }

            // transformation estimation
            Logger::log("Transformation estimation...");
            auto *te = new DefaultTransformationEstimator;
            std::vector<Eigen::Matrix4f> transformations;
            bool tes = te->estimate(
                        source, target,
                        source_keypoints, target_keypoints,
                        source_features, target_features,
                        correspondences, transformations);
            Logger::debug(boost::format("Transformation estimation successfull? %1%") % tes);
            if (tes)
            {
                Logger::debug(boost::format("Clusters: %1%") % transformations.size());

                if (!transformations.empty())
                {
                    for (size_t i = 0; i < transformations.size(); i++)
                    {
                        Logger::debug(boost::format("Instance #%1%:\n%2%") % (i+1) % transformations[i]);
                        PointCloud vpc(source);
                        vpc.transform(transformations[i]);
                        VisualizerObject vpco = Visualizer::visualize(vpc, Color::RED);
                        vpco.setPointSize(3.0);
                    }
                }
            }

            delete te;

            // pose refinement
            //TODO implement
        }

        void useModule(PipelineModuleType::Type module, bool status)
        {
            _usedModules[module] = status;
            //TODO sanity checks, i.e. some modules depend on each other
        }

    private:
        std::shared_ptr<FeatureDescriptor<PointType, DescriptorT> > _featureDescriptor;
        std::shared_ptr<Downsampler<PointType> > _downsampler;
        std::shared_ptr<KeypointExtractor<PointType> > _keypointExtractor;
        std::shared_ptr<TransformationEstimator<PointType, DescriptorT> > _transformationEstimator;
        std::shared_ptr<FeatureMatcher<DescriptorType> > _featureMatcher;

        std::map<PipelineModuleType::Type, bool> _usedModules;
    };
}

#endif // Pipeline_H
