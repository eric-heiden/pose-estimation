#ifndef SHOT_H
#define SHOT_H

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
                              const PclNormalCloud::Ptr &normals,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            Logger::debug("SHOT 1");
            Logger::debug(boost::format("sr: %1%") % (pc.resolution() * searchRadius.value<float>()));
            _shot->setRadiusSearch(pc.resolution() * searchRadius.value<float>());
            Logger::debug("SHOT 2");
            Logger::debug(boost::format("lr: %1%") % (pc.resolution() * lrfRadius.value<float>()));
            _shot->setLRFRadius(pc.resolution() * lrfRadius.value<float>());
            Logger::debug("SHOT 3");
            Logger::debug(boost::format("Keypoints: %1%") % keypoints->size());
            _shot->setInputCloud(keypoints);
            Logger::debug("SHOT 4");
            _shot->setInputNormals(normals);
            Logger::debug("SHOT 5");
            _shot->setSearchSurface(pc.cloud());

            Logger::debug("Start shot");

            Logger::tic("SHOT Feature Extraction");
            _shot->compute(*descriptors);
            Logger::toc("SHOT Feature Extraction");
        }

        static ParameterCategory argumentCategory;

        static Parameter useColor;
        static Parameter searchRadius;
        static Parameter lrfRadius;

    private:
        pcl::SHOTColorEstimationOMP<PointT, NormalType, DescriptorType> *_shot;
    };

    template<typename PointT>
    ParameterCategory SHOTFeatureDescriptor<PointT>::argumentCategory(
            "SHOT", "Feature Extraction using Signature of Histograms of OrienTations (SHOT)");

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

#endif // SHOT_H
