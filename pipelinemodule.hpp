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

        virtual ParameterCategory parameterCategory() const
        {
            return ParameterCategory::EmptyCategory();
        }

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
    virtual ParameterCategory parameterCategory() const \
    { \
        Logger::debug("Overloaded parameter category gets called"); \
        return category; \
    }
}
