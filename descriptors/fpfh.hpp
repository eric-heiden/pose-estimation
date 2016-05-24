#pragma once

#include <pcl/features/fpfh.h>

#include "../types.h"
#include "../parameter.h"
#include "../featuredescription.hpp"
#include "../utils.hpp"

namespace PoseEstimation
{
    /**
     * @brief Feature Description using Fast Point Feature Histograms (FPFH)
     */
    template<typename PointT>
    class FPFHFeatureDescriptor : public FeatureDescriptor<PointT, pcl::FPFHSignature33>
    {
    public:
        typedef pcl::FPFHSignature33 DescriptorType;

        FPFHFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &normals,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            typename pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
            _fpfh.setInputCloud(keypoints);
            _fpfh.setInputNormals(normals);
            _fpfh.setSearchMethod(kdtree);
            _fpfh.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            Logger::tic("FPFH Feature Extraction");
            _fpfh.compute(*descriptors);
            Logger::toc("FPFH Feature Extraction");
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter searchRadius;

    private:        
        pcl::FPFHEstimation<PointT, NormalType, DescriptorType> _fpfh;
    };

    template<typename PointT>
    ParameterCategory FPFHFeatureDescriptor<PointT>::argumentCategory(
            "FPFH", "Feature description using Fast Point Feature Histogram (FPFH)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter FPFHFeatureDescriptor<PointT>::searchRadius(
            "FPFH", "search_r", (float)33.41f,
            "Search radius for finding neighbors",
            { std::make_shared<VariableConstraint>(
              ParameterConstraintType::GreaterThan, "pc_normal_nn")
            });
}
