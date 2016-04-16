#ifndef ICP_H
#define ICP_H

#include <pcl/registration/icp.h>

#include "../poserefinement.hpp"
#include "../parameter.h"
#include "../logger.h"

namespace PoseEstimation
{
    /**
     * @brief Pose refinement using Iterative Closest Point (ICP).
     */
    template<typename PointT>
    class ICPPoseRefiner : PoseRefiner<PointT>
    {
    public:
        virtual bool refine(const PC<PointT> &source, const PC<PointT> &target,
                            PC<PointT> &out, Eigen::Matrix4f &transformation)
        {
            _icp.setInputSource(source.cloud());
            _icp.setInputTarget(target.cloud());

            //TODO use as proper console argument?
            _icp.setUseReciprocalCorrespondences(ConsoleArgument::getOrDefault<bool>("icp_recip_corrs", true));

            // correspondences with higher distances will be ignored
            _icp.setMaxCorrespondenceDistance(
                        source.resolution() * maxCorrespondenceDistance.value<float>()
            );
            // maximum number of iterations (criterion 1)
            _icp.setMaximumIterations(maxIterations.value<int>());
            // transformation epsilon (criterion 2)
            _icp.setTransformationEpsilon(transformationEpsilon.value<float>());
            // euclidean distance difference epsilon (criterion 3)
            _icp.setEuclideanFitnessEpsilon(euclideanFitnessEpsilon.value<float>());

            _icp.align(*(out.cloud()));
            transformation = _icp.getFinalTransformation();

            Logger::debug(boost::format("ICP: -hasConverged? %1%   -fitnessScore = %2%")
                            % boost::io::group(std::boolalpha, _icp.hasConverged())
                            % _icp.getFitnessScore());

            return _icp.hasConverged();
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument maxCorrespondenceDistance;
        static ConsoleArgument maxIterations;
        static ConsoleArgument transformationEpsilon;
        static ConsoleArgument euclideanFitnessEpsilon;

    private:
        pcl::IterativeClosestPoint<PointT, PointT> _icp;
    };

    template<typename PointT>
    ConsoleArgumentCategory ICPPoseRefiner<PointT>::argumentCategory(
                "icp", "Pose refinement using Iterative Closest Point");

    template<typename PointT>
    ConsoleArgument ICPPoseRefiner<PointT>::maxCorrespondenceDistance = ConsoleArgument(
                "icp",
                "corrs_thresh",
                2.0f,
                "Maximum distance threshold between two correspondent points in source <-> target");

    template<typename PointT>
    ConsoleArgument ICPPoseRefiner<PointT>::maxIterations = ConsoleArgument(
                "icp",
                "iter",
                50,
                "Maximum number of iterations");

    template<typename PointT>
    ConsoleArgument ICPPoseRefiner<PointT>::transformationEpsilon = ConsoleArgument(
                "icp",
                "trans_eps",
                (float)1e-8,
                "Maximum allowable difference between two consecutive transformations");

    template<typename PointT>
    ConsoleArgument ICPPoseRefiner<PointT>::euclideanFitnessEpsilon = ConsoleArgument(
                "icp",
                "fit_eps",
                1.0f,
                "Maximum allowed Euclidean error between two consecutive steps in the ICP loop");
}

#endif // ICP_H
