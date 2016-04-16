#ifndef NDT_H
#define NDT_H

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

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument stepSize;
        static ConsoleArgument resolution;
        static ConsoleArgument maxIterations;
        static ConsoleArgument transformationEpsilon;
    };

    template<typename PointT>
    ConsoleArgumentCategory NDTPoseRefiner<PointT>::argumentCategory(
                "ndt", "Pose refinement using Normal Distributions Transform");

    template<typename PointT>
    ConsoleArgument NDTPoseRefiner<PointT>::stepSize = ConsoleArgument(
                "ndt",
                "step",
                0.05f,
                "Maximum step size for More-Thuente line search");

    template<typename PointT>
    ConsoleArgument NDTPoseRefiner<PointT>::resolution = ConsoleArgument(
                "ndt",
                "res",
                0.01f,
                "Resolution of NDT grid structure (VoxelGridCovariance)");

    template<typename PointT>
    ConsoleArgument NDTPoseRefiner<PointT>::maxIterations = ConsoleArgument(
                "ndt",
                "iter",
                2,
                "Maximum number of iterations");

    template<typename PointT>
    ConsoleArgument NDTPoseRefiner<PointT>::transformationEpsilon = ConsoleArgument(
                "ndt",
                "trans_eps",
                0.0001f,
                "Minimum allowable difference between two consecutive transformations");
}

#endif // NDT_H
