#ifndef PipelineModule_H
#define PipelineModule_H

#include <pcl/correspondence.h>

#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    namespace PipelineModuleType {
        /**
         * @brief Enumerates the main components of the Pose Estimation pipeline.
         */
        enum Type
        {
            Downsampler,
            KeypointExtractor,
            FeatureDescriptor,
            FeatureMatcher,
            PoseRefiner,
            TransformationEstimator
        };
    }

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

    private:
        const PipelineModuleType::Type _type;
    };
}

#endif // PipelineModule_H
