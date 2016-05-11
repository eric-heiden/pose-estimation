#pragma once

#include "parameter.h"
#include "pipelinemoduletype.hpp"

namespace PoseEstimation
{
    /**
     * @brief Abstract class that generalizes a module
     * in the Pose Estimation pipeline.
     */
    class PipelineModule
    {
    public:
        PipelineModule(PipelineModuleType::Type t) : _type(t)
        {}

        virtual ParameterCategory parameterCategory() const = 0;

        std::vector<Parameter*> parameters() const
        {
            return parameterCategory().parameters();
        }

        PipelineModuleType::Type type() const
        {
            return _type;
        }

    private:
        const PipelineModuleType::Type _type;
    };

#define PARAMETER_CATEGORY_GETTER(category) \
    ParameterCategory parameterCategory() const \
    { \
        return category; \
    }
}
