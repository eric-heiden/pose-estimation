#pragma once

#include <pcl/features/rsd.h>

#include "../types.h"
#include "../parameter.h"
#include "../featuredescription.hpp"

namespace PoseEstimation
{    
    /**
     * @brief Feature Description using Radius-based Surface Descriptor (RSD)
     */
    template<typename PointT>
    class RSDFeatureDescriptor : public FeatureDescriptor<PointT, pcl::PrincipalRadiiRSD>
    {
    public:
        typedef pcl::PrincipalRadiiRSD DescriptorType;

        RSDFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
            argumentCategory.define();

            typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
            _rsd.setSearchMethod(kdtree);
        }

        virtual void describe(PC<PointT> &pc,
                             const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &normals,
                             pcl::PointCloud<DescriptorType>::Ptr &features)
        {
            _rsd.setInputCloud(keypoints);
            _rsd.setInputNormals(normals);
            _rsd.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _rsd.setPlaneRadius(pc.resolution() * planeRadius.value<float>());
            _rsd.setSaveHistograms(saveHistograms.value<bool>());

            Logger::tic("RSD Feature Extraction");
            _rsd->compute(*features);
            Logger::toc("RSD Feature Extraction");
        }

        static ParameterCategory argumentCategory;

        static Parameter saveHistograms;
        static Parameter searchRadius;
        static Parameter planeRadius;

    private:
        pcl::RSDEstimation<PointT, NormalType, DescriptorType> _rsd;
    };

    template<typename PointT>
    ParameterCategory RSDFeatureDescriptor<PointT>::argumentCategory(
            "RSD", "Feature Description using Radius-based Surface Descriptor (RSD)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter RSDFeatureDescriptor<PointT>::saveHistograms = Parameter(
            "RSD", "save_hist", (bool)false, "Whether to save histograms");

    template<typename PointT>
    Parameter RSDFeatureDescriptor<PointT>::searchRadius = Parameter(
            "RSD", "search_r", (float)5.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");

    template<typename PointT>
    Parameter RSDFeatureDescriptor<PointT>::planeRadius = Parameter(
            "RSD", "plane_r", (float)30.0f, "Maximum radius, above which everything can be considered planar (should be 10-20 times RSD_search_r)");
}
