#pragma once

#include <memory>

#include "keypointextractors/uniform.hpp"
#include "descriptors/shot.hpp"
#include "descriptors/usc.hpp"
#include "descriptors/fpfh.hpp"
#include "descriptors/rift.hpp"
#include "descriptors/rsd.hpp"
#include "descriptors/spinimage.hpp"
#include "transformationestimators/hough3d.hpp"
#include "transformationestimators/gc.hpp"
#include "lrfestimators/board.hpp"
#include "featurematcher/kdtree.hpp"
#include "downsamplers/voxelgrid.hpp"

#include "downsampling.hpp"
#include "keypointextraction.hpp"
#include "featuredescription.hpp"
#include "featurematching.hpp"

namespace PoseEstimation
{
    /*****************************************************************
     * Default Types for the Pose Estimation Pipeline Initialization *
     *****************************************************************/
    typedef VoxelGridDownsampler<PointType> DefaultDownsampler;
    typedef UniformKeypointExtractor<PointType> DefaultKeypointExtractor;
    typedef BOARDLocalReferenceFrameEstimator<PointType> DefaultLRFEstimator;
    typedef SHOTFeatureDescriptor<PointType> DefaultFeatureDescriptor;
    typedef DefaultFeatureDescriptor::DescriptorType DescriptorType;
    typedef GeometricConsistency<PointType, DescriptorType> DefaultTransformationEstimator;
    typedef KdTreeFeatureMatcher<DescriptorType> DefaultFeatureMatcher;
}
