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

        PipelineModuleType::Type type() const
        {
            return _type;
        }

        std::vector<Parameter> parameters() const
        {
            return _parameters;
        }

    protected:
        void _registerParameters(std::initializer_list<Parameter> parameters)
        {
            _parameters.insert(_parameters.end(), parameters.begin(), parameters.end());
        }

    private:
        const PipelineModuleType::Type _type;
        std::vector<Parameter> _parameters;
    };
}
