#pragma once

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
    class ICPPoseRefiner : public PoseRefiner<PointT>
    {
    public:
        ICPPoseRefiner() : PoseRefiner<PointT>()
        {
        }

        virtual bool refine(PC<PointT> &source, PC<PointT> &target,
                            PC<PointT> &out, Eigen::Matrix4f &transformation)
        {
            _icp.setInputSource(source.cloud());
            _icp.setInputTarget(target.cloud());

            //TODO use as proper console argument?
            _icp.setUseReciprocalCorrespondences(Parameter::getOrDefault<bool>("icp_recip_corrs", true));

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

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter maxCorrespondenceDistance;
        static Parameter maxIterations;
        static Parameter transformationEpsilon;
        static Parameter euclideanFitnessEpsilon;

    private:
        pcl::IterativeClosestPoint<PointT, PointT> _icp;
    };

    template<typename PointT>
    ParameterCategory ICPPoseRefiner<PointT>::argumentCategory(
                "icp", "Pose refinement using Iterative Closest Point",
                PipelineModuleType::PoseRefiner);

    template<typename PointT>
    Parameter ICPPoseRefiner<PointT>::maxCorrespondenceDistance = Parameter(
                "icp",
                "corrs_thresh",
                2.0f,
                "Maximum distance threshold between two correspondent points in source <-> target");

    template<typename PointT>
    Parameter ICPPoseRefiner<PointT>::maxIterations = Parameter(
                "icp",
                "iter",
                50,
                "Maximum number of iterations");

    template<typename PointT>
    Parameter ICPPoseRefiner<PointT>::transformationEpsilon = Parameter(
                "icp",
                "trans_eps",
                (float)1e-8,
                "Maximum allowable difference between two consecutive transformations");

    template<typename PointT>
    Parameter ICPPoseRefiner<PointT>::euclideanFitnessEpsilon = Parameter(
                "icp",
                "fit_eps",
                1.0f,
                "Maximum allowed Euclidean error between two consecutive steps in the ICP loop");
}
