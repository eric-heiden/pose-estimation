#ifndef PoseRefinement_H
#define PoseRefinement_H

#include <Eigen/Geometry>

#include "types.h"
#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for registering two point clouds.
     */
    template<typename PointT>
    class PoseRefiner
    {
    public:
        /**
         * @brief Align the source point cloud to the target point cloud.
         * @param source Point cloud to be aligned.
         * @param target Point cloud to be aligned to.
         * @param out Transformed input point cloud.
         * @param transformation Final rigid transformation that aligns source cloud to target.
         * @return Whether the refinement process has converged.
         */
        virtual bool refine(const PC<PointT> &source, const PC<PointT> &target,
                                  PC<PointT> &out, Eigen::Matrix4f &transformation) = 0;
    };
}

#endif // PoseRefinement_H
