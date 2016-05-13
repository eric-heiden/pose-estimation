#pragma once

#include <pcl/features/spin_image.h>

#include "../types.h"
#include "../parameter.h"
#include "../featuredescription.hpp"
#include "../utils.hpp"

namespace PoseEstimation
{
    /**
     * @brief Feature Description using Spin Images (SI).
     */
    template<typename PointT>
    class SpinImageFeatureDescriptor : public FeatureDescriptor<PointT, pcl::Histogram<153> >
    {
    public:
        typedef pcl::Histogram<153> DescriptorType;

        /**
         * @brief Number of bins along one dimension. Changing this value
         * affects the size of the feature descriptor.
         */
        static const int Resolution = 8;

        SpinImageFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
            _si.setImageWidth(Resolution);
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            _si.setInputCloud(keypoints);
            _si.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            Logger::tic("SI Feature Extraction");
            _si.compute(*descriptors);
            Logger::toc("SI Feature Extraction");
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter searchRadius;
        static Parameter minRadius;
        static Parameter lrfRadius;
        static Parameter densityRadius;

    private:
        pcl::SpinImageEstimation<PointT, NormalType, DescriptorType> _si;
    };

    template<typename PointT>
    ParameterCategory SpinImageFeatureDescriptor<PointT>::argumentCategory(
            "SI", "Feature Description using Spin Images (SI)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter SpinImageFeatureDescriptor<PointT>::searchRadius = Parameter(
            "SI", "search_r", (float)5.0f, "Search radius for finding neighbors",
            { std::make_shared<VariableConstraint>(
              ParameterConstraintType::GreaterThan, "pc_normal_nn")
            });
}
