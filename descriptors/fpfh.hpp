#ifndef FPFH_H
#define FPFH_H

#include <pcl/features/fpfh.h>

#include "../types.h"
#include "../consoleargument.h"
#include "../featuredescription.hpp"

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

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            pcl::search::KdTree<PointT>::Ptr kdtree(new pcl::search::KdTree<PointT>);
            _fpfh.setInputCloud(pc.cloud());
            _fpfh.setInputNormals(pc.normals());
            _fpfh.setSearchMethod(kdtree);
            _fpfh.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            Logger::tic("FPFH Feature Extraction");
            _fpfh.compute(*descriptors);
            Logger::toc("FPFH Feature Extraction");
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument searchRadius;

    private:        
        pcl::FPFHEstimation<PointT, NormalType, DescriptorType> _fpfh;
    };

    template<typename PointT>
    ConsoleArgumentCategory FPFHFeatureDescriptor<PointT>::argumentCategory(
            "FPFH", "Fast Point Feature Histogram (FPFH)");

    template<typename PointT>
    ConsoleArgument FPFHFeatureDescriptor<PointT>::searchRadius = ConsoleArgument(
            "FPFH", "search_r", (float)20.0f,
            "Search radius for finding neighbors, must be larger than pc_normal_nn");
}

#endif // FPFH_H
