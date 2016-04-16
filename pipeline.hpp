#ifndef Pipeline_H
#define Pipeline_H

#include "types.h"
#include "defaults.h"
#include "logger.h"
#include "parameter.h"
#include "pointcloud.h"
#include "keypointextraction.hpp"
#include "visualizer.h"

namespace PoseEstimation
{
    /**
     * @brief The standard pose estimation pipeline.
     * @details To estimate the absolute transformation of the source point cloud to the target,
     * the following steps are executed:
     * * (if necessary) downsampling of both point clouds
     * * keypoint extraction
     * * feature description at keypoints
     * * matching of feature descriptors to find correspondences
     * * rigid transformation estimation using corresponding feature descriptors
     * * (if necessary) iterative pose refinement based on priorly estimated transformation
     */
    class Pipeline
    {
    public:
        static void process(PointCloud &source, PointCloud &target)
        {
            // downsample
            Logger::log("Downsampling...");
            DefaultDownsampler ds;
            ds.downsample(source, source);
            ds.downsample(target, target);

            // keypoint extraction
            Logger::log("Keypoint extraction...");
            DefaultKeypointExtractor ke;
            pcl::PointCloud<PointType>::Ptr source_keypoints(new pcl::PointCloud<PointType>);
            pcl::PointCloud<PointType>::Ptr target_keypoints(new pcl::PointCloud<PointType>);
            ke.extract(source, source_keypoints);
            ke.extract(target, target_keypoints);

            // visualize keypoints
            PointCloud skp(source_keypoints);
            VisualizerObject vo = Visualizer::visualize(skp, Color::BLUE);
            vo.setPointSize(5.0);

            PointCloud tkp(target_keypoints);
            vo = Visualizer::visualize(tkp, Color::BLUE);
            vo.setPointSize(5.0);

            // feature description
            Logger::log("Feature description...");
            DefaultFeatureDescriptor fd;
            pcl::PointCloud<DescriptorType>::Ptr source_features(new pcl::PointCloud<DescriptorType>);
            pcl::PointCloud<DescriptorType>::Ptr target_features(new pcl::PointCloud<DescriptorType>);
            fd.describe(source, source_keypoints, source_features);
            Logger::debug(boost::format("Found %d features for source point cloud.") % source_features->size());
            fd.describe(target, target_keypoints, target_features);
            Logger::debug(boost::format("Found %d features for target point cloud.") % target_features->size());

            // matching
            Logger::log("Feature matching...");
            DefaultFeatureMatcher fm;
            pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
            fm.match(source_features, target_features, correspondences);

            for (size_t j = 0; j < correspondences->size(); ++j)
            {
                PointType &model_point = source_keypoints->at((*correspondences)[j].index_query);
                PointType &scene_point = target_keypoints->at((*correspondences)[j].index_match);

                // draw line for each pair of clustered correspondences found between the model and the scene
                Visualizer::visualize(model_point, scene_point, Color::random());
            }

            // transformation estimation
            Logger::log("Transformation estimation...");
            TransformationEstimator<PointType, DescriptorType> *te = new DefaultTransformationEstimator;
            std::vector<Eigen::Matrix4f> transformations;
            bool tes = te->estimate(source, target, source_keypoints, target_keypoints, source_features, target_features, correspondences, transformations);
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

        }
    };
}

#endif // Pipeline_H
