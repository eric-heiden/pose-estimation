#ifndef USC_H
#define USC_H

#include <pcl/features/usc.h>

#include "../types.h"
#include "../consoleargument.h"
#include "../featuredescription.hpp"

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

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            _usc.setInputCloud(pc.cloud());
            _usc.setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _usc.setMinimalRadius(pc.resolution() * minRadius.value<float>());
            _usc.setPointDensityRadius(pc.resolution() * densityRadius.value<float>());
            _usc.setLocalRadius(pc.resolution() * lrfRadius.value<float>());

            Logger::tic("USC Feature Extraction");
            _usc.compute(*descriptors);
            Logger::toc("USC Feature Extraction");
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument searchRadius;
        static ConsoleArgument minRadius;
        static ConsoleArgument lrfRadius;
        static ConsoleArgument densityRadius;

    private:
        pcl::UniqueShapeContext<PointT, DescriptorType, RFType> _usc;
    };

    template<typename PointT>
    ConsoleArgumentCategory USCFeatureDescriptor<PointT>::argumentCategory(
            "USC", "Feature description using Unique Shape Context (USC)";

    template<typename PointT>
    ConsoleArgument USCFeatureDescriptor<PointT>::searchRadius = ConsoleArgument(
            "USC", "search_r", (float)5.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");

    template<typename PointT>
    ConsoleArgument USCFeatureDescriptor<PointT>::lrfRadius = ConsoleArgument(
            "USC", "LRF_r", (float)5.0f, "Local Reference Frame (LRF) radius of USC descriptor");

    template<typename PointT>
    ConsoleArgument USCFeatureDescriptor<PointT>::minRadius = ConsoleArgument(
            "USC", "min_r", (float)1.0f, "Minimum radius of the search sphere");

    template<typename PointT>
    ConsoleArgument USCFeatureDescriptor<PointT>::densityRadius = ConsoleArgument(
            "USC", "density_r", (float)3.0f, "Points within this radius are used to calculate the local point density");
}

#endif // USC_H
