#pragma once

#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include "../types.h"
#include "../parameter.h"
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

        RIFTFeatureDescriptor() : FeatureDescriptor<PointT, DescriptorType>()
        {
            _rift.setNrDistanceBins(DistanceBins);
            _rift.setNrGradientBins(GradientBins);
        }

        virtual void describe(PC<PointT> &pc,
                              const typename pcl::PointCloud<PointT>::Ptr &keypoints,
                              const PclNormalCloud::Ptr &,
                              pcl::PointCloud<DescriptorType>::Ptr &descriptors)
        {
            // convert cloud to intensity cloud
            pcl::PointCloud<pcl::PointXYZI>::Ptr intensity(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::copyPointCloud(*keypoints, *intensity);

            // compute intensity gradients
            pcl::IntensityGradientEstimation<pcl::PointXYZI, NormalType, pcl::IntensityGradient,
                pcl::common::IntensityFieldAccessor<pcl::PointXYZI> > ge;
            ge.setInputCloud(intensity);

            // normal estimation of intensity cloud
            pcl::PointCloud<NormalType>::Ptr normals(new pcl::PointCloud<NormalType>);
            pcl::NormalEstimationOMP<pcl::PointXYZI, NormalType> ne;
            ne.setInputCloud(intensity);
            pcl::search::KdTree<pcl::PointXYZI>::Ptr ne_tree(new pcl::search::KdTree<pcl::PointXYZI>(false));
            ne.setSearchMethod(ne_tree);
            ne.setRadiusSearch(pc.resolution() * normalRadius.value<float>());
            ne.compute(*normals);
            pcl::PointCloud<NormalType>::ConstPtr cnormals(normals);
            ge.setInputNormals(cnormals);

            ge.setRadiusSearch(pc.resolution() * gradientRadius.value<float>());

            pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients(new pcl::PointCloud<pcl::IntensityGradient>);
            ge.compute(*gradients);

            // RIFT feature extraction
            _rift.setInputCloud(intensity);
            typename pcl::search::KdTree<pcl::PointXYZI>::Ptr rift_tree(new pcl::search::KdTree<pcl::PointXYZI>);
            _rift.setSearchMethod(rift_tree);
            _rift.setInputGradient(gradients);
            _rift.setRadiusSearch(pc.resolution() * searchRadius.value<float>());

            Logger::tic("RIFT Feature Extraction");
            _rift.compute(*descriptors);
            Logger::toc("RIFT Feature Extraction");
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter gradientRadius;
        static Parameter searchRadius;
        static Parameter normalRadius;

    private:
        pcl::RIFTEstimation<pcl::PointXYZI, pcl::IntensityGradient, DescriptorType> _rift;
    };

    template<typename PointT>
    ParameterCategory RIFTFeatureDescriptor<PointT>::argumentCategory(
            "RIFT", "Feature description using Rotation Invariant Feature Transform (RIFT)",
            PipelineModuleType::FeatureDescriptor);

    template<typename PointT>
    Parameter RIFTFeatureDescriptor<PointT>::gradientRadius = Parameter(
            "RIFT", "gradient_r", (float)50.0f, "Search radius for intensity gradient computation");

    template<typename PointT>
    Parameter RIFTFeatureDescriptor<PointT>::searchRadius = Parameter(
            "RIFT", "search_r", (float)50.0f, "Search radius for finding neighbors",
            { std::make_shared<VariableConstraint>(
              ParameterConstraintType::GreaterThan, "pc_normal_nn")
            });

    template<typename PointT>
    Parameter RIFTFeatureDescriptor<PointT>::normalRadius = Parameter(
            "RIFT", "normal_nn", (float)20.0f, "Search radius for finding neighbors for normal estimation");
}
