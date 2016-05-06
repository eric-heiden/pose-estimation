#pragma once

#include <pcl/registration/ndt.h>

#include "../poserefinement.hpp"
#include "../parameter.h"
#include "../logger.h"

namespace PoseEstimation
{
    /**
     * @brief Pose refinement using Normal Distributions Transform (NDT).
     */
    template<typename PointT>
    class NDTPoseRefiner : PoseRefiner<PointT>
    {
    public:
        NDTPoseRefiner() : PoseRefiner<PointT>()
        {
            argumentCategory.define();
        }

        virtual bool refine(const PC<PointT> &source, const PC<PointT> &target,
                            PC<PointT> &out, Eigen::Matrix4f &transformation)
        {
            pcl::NormalDistributionsTransform<PointT, PointT> ndt;

            // Setting minimum transformation difference for termination condition.
            ndt.setTransformationEpsilon(transformationEpsilon.value<float>());
            // Setting maximum step size for More-Thuente line search.
            ndt.setStepSize(stepSize.value<float>());
            // Setting Resolution of NDT grid structure (VoxelGridCovariance).
            ndt.setResolution(resolution.value<float>());

            ndt.setMaximumIterations(maxIterations.value<int>());

            ndt.setInputSource(source.cloud());
            ndt.setInputTarget(target.cloud());

            ndt.align(*(out.cloud()));
            transformation = ndt.getFinalTransformation();

            Logger::debug(boost::format("NDT: -hasConverged=%1%   -fitnessScore=%2%")
                            % boost::io::group(std::boolalpha, ndt.hasConverged())
                            % ndt.getFitnessScore());

            return ndt.hasConverged();
        }

        static ParameterCategory argumentCategory;

        static Parameter stepSize;
        static Parameter resolution;
        static Parameter maxIterations;
        static Parameter transformationEpsilon;
    };

    template<typename PointT>
    ParameterCategory NDTPoseRefiner<PointT>::argumentCategory(
                "ndt", "Pose refinement using Normal Distributions Transform",
                PipelineModuleType::PoseRefiner);

    template<typename PointT>
    Parameter NDTPoseRefiner<PointT>::stepSize = Parameter(
                "ndt",
                "step",
                0.05f,
                "Maximum step size for More-Thuente line search");

    template<typename PointT>
    Parameter NDTPoseRefiner<PointT>::resolution = Parameter(
                "ndt",
                "res",
                0.01f,
                "Resolution of NDT grid structure (VoxelGridCovariance)");

    template<typename PointT>
    Parameter NDTPoseRefiner<PointT>::maxIterations = Parameter(
                "ndt",
                "iter",
                2,
                "Maximum number of iterations");

    template<typename PointT>
    Parameter NDTPoseRefiner<PointT>::transformationEpsilon = Parameter(
                "ndt",
                "trans_eps",
                0.0001f,
                "Minimum allowable difference between two consecutive transformations");
}
