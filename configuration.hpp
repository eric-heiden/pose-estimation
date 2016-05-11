#pragma once

#include "pipelinemodule.hpp"
#include "parameter.h"
#include "pipeline.hpp"

namespace PoseEstimation
{
    /**
     * @brief Configuration of modules to use for pose estimation pipeline
     */
    template<class PointT>
    class CF : public PipelineModule
    {
    public:
        CF() : PipelineModule(PipelineModuleType::Miscellaneous)
        {}

        PipelineStats run(PC<PointT> &source, PC<PointT> &target)
        {
            // execute pipeline::process
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static EnumParameter downsampler;
    };

    template<class PointT>
    ParameterCategory CF<PointT>::argumentCategory(
            "config", "Configuration of modules to use for pose estimation pipeline",
            PipelineModuleType::Miscellaneous);

    template<typename PointT>
    EnumParameter CF<PointT>::downsampler = EnumParameter(
            "config", "downsampler",
            { "uniform", "voxelgrid" },
            "Downsampler module");

    typedef CF<PointType> Configuration;
}
