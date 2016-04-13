#ifndef SHOT_H
#define SHOT_H

#include <pcl/features/shot_omp.h>

#include "../consoleargument.h"
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

        SHOTFeatureDescriptor()
        {
            //TODO: initialize in describe(...) to make useColor runtime variable?
            _shot = new pcl::SHOTColorEstimationOMP
                    <PointT, NormalType, DescriptorType>(true, useColor.value<bool>());
        }

        virtual ~SHOTFeatureDescriptor()
        {
            delete _shot;
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            _shot->setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            _shot->setLRFRadius(pc.resolution() * lrfRadius.value<float>());
            _shot->setInputCloud(keypoints);
            _shot->setInputNormals(pc.normals());
            _shot->setSearchSurface(pc.cloud());

            Logger::tic("SHOT Feature Extraction");
            _shot->compute(*descriptors);
            Logger::toc("SHOT Feature Extraction");
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument useColor;
        static ConsoleArgument searchRadius;
        static ConsoleArgument lrfRadius;

    private:
        pcl::SHOTColorEstimationOMP<PointT, NormalType, DescriptorType> *_shot;
    };

    template<typename PointT>
    ConsoleArgumentCategory SHOTFeatureDescriptor<PointT>::argumentCategory(
            "SHOT", "Feature Extraction using Signature of Histograms of OrienTations (SHOT)");

    template<typename PointT>
    ConsoleArgument SHOTFeatureDescriptor<PointT>::useColor = ConsoleArgument(
            "SHOT", "color", (bool)false, "Consider color information");

    template<typename PointT>
    ConsoleArgument SHOTFeatureDescriptor<PointT>::searchRadius = ConsoleArgument(
            "SHOT", "search_r", (float)15.0f, "Search radius for finding neighbors, must be larger than pc_normal_nn");

    template<typename PointT>
    ConsoleArgument SHOTFeatureDescriptor<PointT>::lrfRadius = ConsoleArgument(
            "SHOT", "LRF_r", (float)27.5f, "Local Reference Frame (LRF) radius of SHOT descriptor");
}

#endif // SHOT_H
