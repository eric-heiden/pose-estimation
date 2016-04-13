#ifndef RSD_H
#define RSD_H

#include <pcl/features/rsd.h>

#include "../types.h"
#include "../consoleargument.h"
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

        RSDFeatureDescriptor()
        {
            pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
            _rsd.setSearchMethod(kdtree);
        }

        virtual void describe(PC<PointT> &pc,
                             const typename pcl::PointCloud<PointT>::Ptr &,
                             pcl::PointCloud<DescriptorType>::Ptr &features)
        {
            _rsd.setInputCloud(cloud);
            _rsd.setInputNormals(normals);
            _rsd.setRadiusSearch(pc.resolution() * searchRadius);
            // Plane radius. Any radius larger than this is considered infinite (a plane).
            _rsd.setPlaneRadius(0.1);
            // Do we want to save the full distance-angle histograms?
            _rsd.setSaveHistograms(false);

            Logger::tic("RSD Feature Extraction");
            _RSD->compute(*features);
            Logger::toc("RSD Feature Extraction");
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument saveHistograms;
        static ConsoleArgument searchRadius;
        static ConsoleArgument planeRadius;

    private:
        pcl::RSDEstimation<PointT, NormalType, DescriptorType> _rsd;
    };

    template<typename PointT>
    ConsoleArgumentCategory RSDFeatureDescriptor<PointT>::argumentCategory(
            "RSD", "Feature Description using Radius-based Surface Descriptor (RSD)");

    template<typename PointT>
    ConsoleArgument RSDFeatureDescriptor<PointT>::saveHistograms = ConsoleArgument(
            "RSD", "color", (bool)false, "Consider color information");

    template<typename PointT>
    ConsoleArgument RSDFeatureDescriptor<PointT>::searchRadius = ConsoleArgument(
            "RSD", "search_r", (float)5.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");

    template<typename PointT>
    ConsoleArgument RSDFeatureDescriptor<PointT>::planeRadius = ConsoleArgument(
            "RSD", "LRF_r", (float)30.0f, "Maximum radius, above which everything can be considered planar (should be 10-20 times RSD_search_r)");
}

#endif // RSD_H
