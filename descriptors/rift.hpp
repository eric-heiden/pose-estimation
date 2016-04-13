#ifndef RIFT_H
#define RIFT_H

#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>

#include "../types.h"
#include "../consoleargument.h"
#include "../featuredescription.hpp"

namespace PoseEstimation
{
    /**
     * @brief Feature Description using Rotation Invariant Feature Transform (RIFT).
     */
    template<typename PointT>
    class RIFTFeatureDescriptor : public FeatureDescriptor<PointT, pcl::Histogram<32> >
    {
    public:
        // since these values determine the feature size, they cannot be dynamic
        /**
         * @brief Number of bins to use in the distance dimension
         */
        static const int DistanceBins = 4;
        /**
         * @brief Number of bins to use in the gradient orientation dimension
         */
        static const int GradientBins = 8;

        typedef pcl::Histogram<(DistanceBins*GradientBins)> DescriptorType;

        RIFTFeatureDescriptor()
        {
            _rift.setNrDistanceBins(DistanceBins);
            _rift.setNrGradientBins(GradientBins);
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            // convert cloud to intensity cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr intensity(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*(pc.cloud()), *intensity);

            // compute intensity gradients
            pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
            pcl::IntensityGradientEstimation<pcl::PointXYZI, pcl::Normal, pcl::IntensityGradient,
                pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
            ge.setInputCloud(intensity);
            ge.setInputNormals(pc.normals());
            ge.setRadiusSearch(pc.resolution() * gradientRadius.value<float>());
            ge.compute(*gradients);

            _rift.setInputCloud(intensity);
            _rift.setSearchMethod(kdtree);
            _rift.setInputGradient(gradients);
            _rift.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            Logger::tic("RIFT Feature Extraction");
            _rift.compute(*descriptors);
            Logger::toc("RIFT Feature Extraction");
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument gradientRadius;
        static ConsoleArgument searchRadius;

    private:
        pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, DescriptorType> _rift;
    };

    template<typename PointT>
    ConsoleArgumentCategory RIFTFeatureDescriptor<PointT>::argumentCategory(
            "RIFT", "Feature description using Rotation Invariant Feature Transform (RIFT)";

    template<typename PointT>
    ConsoleArgument RIFTFeatureDescriptor<PointT>::gradientRadius = ConsoleArgument(
            "RIFT", "gradient_r", (float)3.0f, "Search radius for intensity gradient computation");

    template<typename PointT>
    ConsoleArgument RIFTFeatureDescriptor<PointT>::searchRadius = ConsoleArgument(
            "RIFT", "search_r", (float)2.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");
}

#endif // RIFT_H
