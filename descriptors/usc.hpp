#pragma once

#include <pcl/features/usc.h>

#include "../types.h"
#include "../parameter.h"
#include "../featuredescription.hpp"
#include "../utils.hpp"

namespace PoseEstimation
{
    /**
     * @brief Feature Description using Unique Shape Context (USC).
     */
    template<typename PointT>
    class USCFeatureDescriptor : public FeatureDescriptor<PointT, pcl::UniqueShapeContext1960>
    {
    public:
        typedef pcl::UniqueShapeContext1960 DescriptorType;

        USCFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
            argumentCategory.define();
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            _usc.setInputCloud(keypoints);
            _usc.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _usc.setMinimalRadius(pc.resolution() * minRadius.value<float>());
            _usc.setPointDensityRadius(pc.resolution() * densityRadius.value<float>());
            _usc.setLocalRadius(pc.resolution() * lrfRadius.value<float>());

            Logger::tic("USC Feature Extraction");
            _usc.compute(*descriptors);
            Logger::toc("USC Feature Extraction");
        }

        static ParameterCategory argumentCategory;

        static Parameter searchRadius;
        static Parameter minRadius;
        static Parameter lrfRadius;
        static Parameter densityRadius;

    private:
        pcl::UniqueShapeContext<PointT, DescriptorType, RFType> _usc;
    };

    template<typename PointT>
    ParameterCategory USCFeatureDescriptor<PointT>::argumentCategory(
            "USC", "Feature description using Unique Shape Context (USC)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter USCFeatureDescriptor<PointT>::searchRadius = Parameter(
            "USC", "search_r", (float)25.0f, "Search radius for finding neighbors",
            { std::make_shared<VariableConstraint>(
              ParameterConstraintType::GreaterThan, "pc_normal_nn")
            });

    template<typename PointT>
    Parameter USCFeatureDescriptor<PointT>::lrfRadius = Parameter(
            "USC", "LRF_r", (float)25.0f, "Local Reference Frame (LRF) radius of USC descriptor");

    template<typename PointT>
    Parameter USCFeatureDescriptor<PointT>::minRadius = Parameter(
            "USC", "min_r", (float)5.0f, "Minimum radius of the search sphere");

    template<typename PointT>
    Parameter USCFeatureDescriptor<PointT>::densityRadius = Parameter(
            "USC", "density_r", (float)10.0f, "Points within this radius are used to calculate the local point density");
}
