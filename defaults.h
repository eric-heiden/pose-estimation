#ifndef DEFAULTS_H
#define DEFAULTS_H

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

namespace PoseEstimation
{
    /**********************************************************
     *  Default modules used by the Pose Estimation pipeline  *
     **********************************************************/
    typedef VoxelGridDownsampler<PointType> DefaultDownsampler;
    typedef UniformKeypointExtractor<PointType> DefaultKeypointExtractor;
    typedef BOARDLocalReferenceFrameEstimator<PointType> DefaultLRFEstimator;
    //typedef SHOTFeatureDescriptor<PointType> DefaultFeatureDescriptor;
    typedef RIFTFeatureDescriptor<PointType> DefaultFeatureDescriptor;
    typedef DefaultFeatureDescriptor::DescriptorType DescriptorType;
    //typedef HoughVoting<DescriptorType, DefaultLRFEstimator> DefaultTransformationEstimator;
    typedef GeometricConsistency<PointType, DescriptorType> DefaultTransformationEstimator;
    typedef KdTreeFeatureMatcher<DescriptorType> DefaultFeatureMatcher;

    ///TODO implement logic to change defaults via {@see ConsoleArgument}s.
}

#endif // DEFAULTS_H
