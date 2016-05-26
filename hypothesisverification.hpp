#pragma once

#include <pcl/point_cloud.h>
#include <pcl/recognition/hv/hv_go.h>

#include "pipelinemodule.hpp"
#include "types.h"
#include "pointcloud.h"
#include "logger.h"

namespace PoseEstimation
{
    /**
     * @brief Global Hypothesis Verification
     */
    template<typename PointT>
    class HypothesisVerifier : public PipelineModule
    {
    public:
        HypothesisVerifier() : PipelineModule(PipelineModuleType::HypothesisVerifier)
        {}

        /**
         * @brief Verifies the given transformation candidates to reduce the number
         * of false positives.
         * @param source The point cloud of the object.
         * @param source The point cloud of the scene.
         * @param transformations The hypotheses.
         * @return Mask vector determining for each transformation whether it fulfills
         * the hypothesis test.
         */
        std::vector<bool> verify(PC<PointT> &source, PC<PointT> &target, std::vector<Eigen::Matrix4f> &transformations)
        {
            std::vector<typename pcl::PointCloud<PointT>::ConstPtr > candidates;
            for (auto &transformation : transformations)
            {
                typename pcl::PointCloud<PointT>::Ptr candidate(new typename pcl::PointCloud<PointT>);
                pcl::transformPointCloud(*(source.cloud()), *candidate, transformation);
                candidates.push_back(candidate);
            }

            pcl::GlobalHypothesesVerification<PointT, PointT> hv;

            hv.setSceneCloud(target.cloud());
            hv.addModels(candidates, true);

            hv.setInlierThreshold(inlierThreshold.value<float>());
            hv.setOcclusionThreshold(occlusionThreshold.value<float>());
            hv.setRegularizer(regularizer.value<float>());
            hv.setRadiusClutter(clutterRadius.value<float>());
            hv.setClutterRegularizer(clutterRegularizer.value<float>());
            hv.setDetectClutter(clutterDetection.value<bool>());
            hv.setRadiusNormals(normalEstimationRadius.value<float>());

            hv.verify();

            std::vector<bool> mask;
            hv.getMask(mask);

            size_t positive = 0;
            for (bool b : mask)
                positive += (size_t)b;

            Logger::debug(boost::format("Hypothesis verification: %i out of %i candidates are positive.")
                          % positive % transformations.size());

            return mask;
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter inlierThreshold;
        static Parameter occlusionThreshold;
        static Parameter regularizer;
        static Parameter clutterRadius;
        static Parameter clutterRegularizer;
        static Parameter clutterDetection;
        static Parameter normalEstimationRadius;
    };

    template<typename PointT>
    ParameterCategory HypothesisVerifier<PointT>::argumentCategory(
            "hv", "Global Hypothesis Verification", PipelineModuleType::HypothesisVerifier);

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::inlierThreshold(
            "hv", "inlier_thresh", (float)0.005f,
            "Inlier threshold",
            NUMERICAL_PARAMETER_RANGE(0.0, 1.0));

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::occlusionThreshold(
            "hv", "occlusion_thresh", (float)0.01f,
            "Occlusion threshold",
            NUMERICAL_PARAMETER_RANGE(0.0, 1.0));

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::regularizer(
            "hv", "regularizer", (float)3.0f,
            "Regularizer value");

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::clutterRadius(
            "hv", "clutter_r", (float)0.03f,
            "Clutter radius",
            NUMERICAL_PARAMETER_RANGE(0.0, 1.0));

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::clutterRegularizer(
            "hv", "clutter_regularizer", (float)2.0f,
            "Clutter regularizer");

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::clutterDetection(
            "hv", "clutter", true,
            "Whether to perform clutter detection");

    template<typename PointT>
    Parameter HypothesisVerifier<PointT>::normalEstimationRadius(
            "hv", "normal_r", (float)0.05f,
            "Search radius for normal estimation",
            NUMERICAL_PARAMETER_RANGE(0.0, 1.0));
}
