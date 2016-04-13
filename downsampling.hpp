#ifndef Downsampling_H
#define Downsampling_H

#include "pointcloud.h"

namespace PoseEstimation
{
    /**
     * @brief Abstract module for downsampling point clouds.
     */
    template<typename PointT>
    class Downsampler
    {
    public:
        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const = 0;
    };
}

#endif // Downsampling_H
