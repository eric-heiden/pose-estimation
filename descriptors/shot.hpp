#pragma once

#include <pcl/features/shot_omp.h>

#include "../parameter.h"
#include "../featuredescription.hpp"

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
            argumentCategory.define();
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &normals,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            static auto shot = pcl::SHOTColorEstimationOMP
                    <PointT, NormalType, DescriptorType>(true, useColor.value<bool>());

            shot.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            shot.setLRFRadius(pc.resolution() * lrfRadius.value<float>());
            shot.setInputCloud(keypoints);
            shot.setInputNormals(pc.normals());
            shot.setSearchSurface(pc.cloud());

            Logger::tic("SHOT Feature Extraction");
            shot.compute(*descriptors);
            Logger::toc("SHOT Feature Extraction");
        }

        static ParameterCategory argumentCategory;

        static Parameter useColor;
        static Parameter searchRadius;
        static Parameter lrfRadius;
    };

    template<typename PointT>
    ParameterCategory SHOTFeatureDescriptor<PointT>::argumentCategory(
            "SHOT", "Feature Extraction using Signature of Histograms of OrienTations (SHOT)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::useColor = Parameter(
            "SHOT", "color", (bool)false, "Consider color information");

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::searchRadius = Parameter(
            "SHOT", "search_r", (float)15.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");

    template<typename PointT>
    Parameter SHOTFeatureDescriptor<PointT>::lrfRadius = Parameter(
            "SHOT", "LRF_r", (float)27.5f, "Local Reference Frame (LRF) radius of SHOT descriptor");
}
