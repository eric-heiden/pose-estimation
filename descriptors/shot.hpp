#pragma once

#include <pcl/features/shot_omp.h>

#include "../parameter.h"
#include "../featuredescription.hpp"
#include "../utils.hpp"

namespace PoseEstimation
{
    /**
     * @brief Feature Description using Signature of Histograms of OrienTations (SHOT)
     */
    template<typename PointT>
    class SHOTFeatureDescriptor : public FeatureDescriptor<PointT, pcl::SHOT1344>
    {
    public:
        typedef pcl::SHOT1344 DescriptorType;

        SHOTFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            static auto shot = pcl::SHOTColorEstimationOMP
                    <PointT, NormalType, DescriptorType>(true, useColor.value<bool>());

            shot.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            // built-in local reference frame estimation using BOrder Aware Repeatable Directions (BOARD)
            shot.setLRFRadius(pc.resolution() * lrfRadius.value<float>());
            shot.setInputCloud(keypoints);
            shot.setInputNormals(pc.normals());
            shot.setSearchSurface(pc.cloud());

            Logger::tic("SHOT Feature Extraction");
            shot.compute(*descriptors);
            Logger::toc("SHOT Feature Extraction");
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter useColor;
        static Parameter searchRadius;
        static Parameter lrfRadius;
    };

    template<typename PointT>
    ParameterCategory SHOTFeatureDescriptor<PointT>::argumentCategory(
            "SHOT", "Feature description using Signature of Histograms of OrienTations (SHOT)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::useColor = Parameter(
            "SHOT", "color", (bool)true, "Consider color information");

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::searchRadius = Parameter(
            "SHOT", "search_r", (float)15.0f, "Search radius for finding neighbors",
            { std::make_shared<VariableConstraint>(
              ParameterConstraintType::GreaterThan, "pc_normal_nn")
            });

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::lrfRadius = Parameter(
            "SHOT", "LRF_r", (float)10.0f, "Local Reference Frame (LRF) radius of SHOT descriptor");
}
