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

        PipelineModule(const PipelineModule &p) : _type(p._type)
        {}

        PipelineModule(PipelineModule &&p) : _type(p._type)
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

        // does nothing, since _type is constant
        PipelineModule& operator=(const PipelineModule&)
        {}

        // does nothing, since _type is constant
        PipelineModule& operator=(PipelineModule&&)
        {}

    private:
        const PipelineModuleType::Type _type;
    };

#define PARAMETER_CATEGORY_GETTER(category) \
    virtual ParameterCategory parameterCategory() const \
    { \
        return category; \
    }
}
