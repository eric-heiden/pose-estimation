#pragma once

#include <boost/preprocessor.hpp>

#include "pipelinemodule.hpp"
#include "parameter.h"
#include "pipeline.hpp"
#include "logger.h"

#include "downsamplers/uniform.hpp"
#include "downsamplers/voxelgrid.hpp"

#include "descriptors/fpfh.hpp"
#include "descriptors/rift.hpp"
#include "descriptors/rsd.hpp"
#include "descriptors/shot.hpp"
#include "descriptors/spinimage.hpp"
#include "descriptors/usc.hpp"

#include "featurematcher/kdtree.hpp"

#include "keypointextractors/iss.hpp"
#include "keypointextractors/uniform.hpp"

#include "transformationestimators/gc.hpp"
#include "transformationestimators/hough3d.hpp"
#include "transformationestimators/ransac.hpp"
#include "transformationestimators/svd.hpp"

namespace PoseEstimation
{
#define DESCRIPTORS (FPFHFeatureDescriptor) \
                    (RIFTFeatureDescriptor) \
                    (RSDFeatureDescriptor) \
                    (SHOTFeatureDescriptor) \
                    (SpinImageFeatureDescriptor) \
                    (USCFeatureDescriptor)

#define CASE_BRANCH_INNER(elem) \
{ \
    std::shared_ptr<FeatureDescriptor<PointT, typename elem<PointT>::DescriptorType> > f = std::make_shared<elem<PointT> >(); \
    return _run<typename elem<PointT>::DescriptorType>(source, target, f); \
}

#define CASE_BRANCH(next, _, i, elem) \
    case i: CASE_BRANCH_INNER(elem)


#define CASE_BODY(seq) \
    BOOST_PP_SEQ_FOR_EACH_I(CASE_BRANCH, _, seq); \
    default: CASE_BRANCH_INNER(BOOST_PP_SEQ_HEAD(seq));


    /**
     * @brief Configuration of modules for pose estimation pipeline
     */
    template<class PointT>
    class CF : public PipelineModule
    {
    public:
        CF() : PipelineModule(PipelineModuleType::Miscellaneous)
        {
            for (auto *parameter : involvedParameters())
            {
                Logger::log(boost::format("Configuration parameter %s") % parameter->parseName());
            }
        }

        PipelineStats run(PC<PointT> &source, PC<PointT> &target)
        {
            switch (descriptor.value<Enum>().value)
            {
                CASE_BODY(DESCRIPTORS)
            }
        }

        //TODO implement LRF Estimator module selector (currently useless as there is only BOARD LRF Estimation).
        //TODO implement pose refiner module selector (currently not used in the pipeline)

        /**
         * @brief Retrieves all parameters of the configured modules.
         * @return
         */
        static std::vector<Parameter*> involvedParameters()
        {
            std::vector<Parameter*> ps;

            _appendParameters(_moduleParameters(descriptor), ps);
            _appendParameters(_moduleParameters(downsampler), ps);
            _appendParameters(_moduleParameters(featureMatcher), ps);
            _appendParameters(_moduleParameters(keypointExtractor), ps);
            _appendParameters(_moduleParameters(transformationEstimator), ps);

            return ps;
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static EnumParameter descriptor;
        static EnumParameter downsampler;
        static EnumParameter featureMatcher;
        static EnumParameter keypointExtractor;
        static EnumParameter transformationEstimator;

    private:
        static void _appendParameters(const std::vector<Parameter*> &source,
                                      std::vector<Parameter*> &target)
        {
            target.insert(target.end(), source.begin(), source.end());
        }

        static std::vector<Parameter*> _moduleParameters(const EnumParameter &module)
        {
            return ParameterCategory(module.value<Enum>().valueName()).parameters();
        }

        template<typename DescriptorT>
        static PipelineStats _run(PC<PointT> &source, PC<PointT> &target,
                                  typename std::shared_ptr<FeatureDescriptor<PointT, DescriptorT> > &descriptor)
        {
            Pipeline<DescriptorT, PointT> pipeline;
            pipeline.featureDescriptor() = descriptor;

            switch (downsampler.value<Enum>().value)
            {
                case 1:
                    pipeline.downsampler() = std::make_shared<VoxelGridDownsampler<PointT> >();
                    break;
                default:
                    pipeline.downsampler() = std::make_shared<UniformDownsampler<PointT> >();
            }

            switch (featureMatcher.value<Enum>().value)
            {
                default:
                    pipeline.featureMatcher() = std::make_shared<KdTreeFeatureMatcher<DescriptorT> >();
            }

            switch (keypointExtractor.value<Enum>().value)
            {
                case 1:
                    pipeline.keypointExtractor() = std::make_shared<ISSKeypointExtractor<PointT> >();
                    break;
                default:
                    pipeline.keypointExtractor() = std::make_shared<UniformKeypointExtractor<PointT> >();
            }

            switch (transformationEstimator.value<Enum>().value)
            {
                case 1:
                    pipeline.transformationEstimator() = std::make_shared<HoughVoting<PointT, DescriptorT> >();
                    break;
                case 2:
                    pipeline.transformationEstimator() = std::make_shared<RANSACTransformationEstimator<PointT, DescriptorT> >();
                    break;
                case 3:
                    pipeline.transformationEstimator() = std::make_shared<SVDTransformationEstimator<PointT, DescriptorT> >();
                    break;
                default:
                    pipeline.transformationEstimator() = std::make_shared<GeometricConsistency<PointT, DescriptorT> >();
            }

            return pipeline.process(source, target);
        }
    };

    template<class PointT>
    ParameterCategory CF<PointT>::argumentCategory(
            "config", "Configuration of modules to use for pose estimation pipeline",
            PipelineModuleType::Miscellaneous);

    template<typename PointT>
    EnumParameter CF<PointT>::descriptor = EnumParameter(
            "config", "descriptor",
            { "FPFH", "RIFT", "RSD", "SHOT", "SI", "USC" },
            "Feature descriptor module");

    template<typename PointT>
    EnumParameter CF<PointT>::downsampler = EnumParameter(
            "config", "downsampler",
            { "uniformdown", "voxelgrid" },
            "Downsampler module");

    template<typename PointT>
    EnumParameter CF<PointT>::featureMatcher = EnumParameter(
            "config", "feature_matcher",
            { "kdmatch" },
            "Feature matcher module");

    template<typename PointT>
    EnumParameter CF<PointT>::keypointExtractor = EnumParameter(
            "config", "keypoint_extractor",
            { "uniform", "iss" },
            "Keypoint extractor module");

    template<typename PointT>
    EnumParameter CF<PointT>::transformationEstimator = EnumParameter(
            "config", "transformation_estimator",
            { "gc", "hough", "ransac", "svd" },
            "Transformation estimator module");


    typedef CF<PointType> Configuration;
}
