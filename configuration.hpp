#pragma once

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
            // configure pipeline and execute pipeline::process
        }

        //TODO implement LRF Estimator module selector (currently useless as there is only BOARD LRF Estimation).
        //TODO implement pose refiner module selector (currently not used in the pipeline)

        template<class T>
        static std::shared_ptr<T> descriptorModule()
        {
            switch (descriptor.value<Enum>().value)
            {
                case 0:
                    return std::make_shared<T>(FPFHFeatureDescriptor<PointT>());
                case 1:
                    return std::make_shared<T>(RIFTFeatureDescriptor<PointT>());
                case 2:
                    return std::make_shared<T>(RSDFeatureDescriptor<PointT>());
                case 3:
                    return std::make_shared<T>(SHOTFeatureDescriptor<PointT>());
                case 4:
                    return std::make_shared<T>(SpinImageFeatureDescriptor<PointT>());
                default:
                    return std::make_shared<T>(USCFeatureDescriptor<PointT>());
            }
        }

        template<class T>
        static std::shared_ptr<T> downsamplerModule()
        {
            switch (downsampler.value<Enum>().value)
            {
                case 0:
                    return std::make_shared<T>(UniformDownsampler<PointT>());
                default:
                    return std::make_shared<T>(VoxelGridDownsampler<PointT>());
            }
        }

        template<class T, typename DescriptorT>
        static std::shared_ptr<T> featureMatcherModule()
        {
            switch (featureMatcher.value<Enum>().value)
            {
                default:
                    return std::make_shared<T>(KdTreeFeatureMatcher<DescriptorT>());
            }
        }

        template<class T>
        static std::shared_ptr<T> keypointExtractorModule()
        {
            switch (keypointExtractor.value<Enum>().value)
            {
                case 0:
                    return std::make_shared<T>(UniformKeypointExtractor<PointT>());
                default:
                    return std::make_shared<T>(ISSKeypointExtractor<PointT>());
            }
        }

        template<class T, typename DescriptorT>
        static std::shared_ptr<T> transformationEstimatorModule()
        {
            switch (transformationEstimator.value<Enum>().value)
            {
                case 0:
                    return std::make_shared<T>(GeometricConsistency<PointT, DescriptorT>());
                case 1:
                    return std::make_shared<T>(HoughVoting<PointT, DescriptorT>());
                case 2:
                    return std::make_shared<T>(RANSACTransformationEstimator<PointT, DescriptorT>());
                default:
                    return std::make_shared<T>(SVDTransformationEstimator<PointT, DescriptorT>());
            }
        }

        /**
         * @brief Retrieves all parameters of the configured modules.
         * @return
         */
        static std::vector<Parameter*> involvedParameters()
        {
            std::vector<Parameter*> ps;

            _appendParameters(descriptorModule<PipelineModule>()->parameters(), ps);
            _appendParameters(downsamplerModule<PipelineModule>()->parameters(), ps);
            _appendParameters(featureMatcherModule<PipelineModule, PointT>()->parameters(), ps);
            _appendParameters(keypointExtractorModule<PipelineModule>()->parameters(), ps);
            _appendParameters(transformationEstimatorModule<PipelineModule, PointT>()->parameters(), ps);

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
    };

    template<class PointT>
    ParameterCategory CF<PointT>::argumentCategory(
            "config", "Configuration of modules to use for pose estimation pipeline",
            PipelineModuleType::Miscellaneous);

    template<typename PointT>
    EnumParameter CF<PointT>::descriptor = EnumParameter(
            "config", "descriptor",
            { "fpfh", "rift", "rsd", "shot", "spinimage", "usc" },
            "Feature descriptor module");

    template<typename PointT>
    EnumParameter CF<PointT>::downsampler = EnumParameter(
            "config", "downsampler",
            { "uniform", "voxelgrid" },
            "Downsampler module");

    template<typename PointT>
    EnumParameter CF<PointT>::featureMatcher = EnumParameter(
            "config", "feature_matcher",
            { "kdtree" },
            "Feature matcher module");

    template<typename PointT>
    EnumParameter CF<PointT>::keypointExtractor = EnumParameter(
            "config", "keypoint_extractor",
            { "uniform", "iss" },
            "Keypoint extractor module");

    template<typename PointT>
    EnumParameter CF<PointT>::transformationEstimator = EnumParameter(
            "config", "transformation_estimator",
            { "gc", "hough3d", "ransac", "svd" },
            "Transformation estimator module");


    typedef CF<PointType> Configuration;
}
