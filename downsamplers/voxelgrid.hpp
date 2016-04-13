#ifndef VoxelGridDownsampling_H
#define VoxelGridDownsampling_H

#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>

#include "../types.h"
#include "../consoleargument.h"
#include "../pointcloud.h"
#include "../downsampling.hpp"

namespace PoseEstimation
{
    template<typename PointT>
    class VoxelGridDownsampler : Downsampler<PointT>
    {
    public:
        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const
        {
            pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
            float sample_size = voxelSize.value<float>() * pc.resolution();
            approximate_voxel_filter.setLeafSize(sample_size, sample_size, sample_size);
            approximate_voxel_filter.setInputCloud(pc.cloud());
            approximate_voxel_filter.filter(*(downsampled.cloud()));
        }

        static ConsoleArgumentCategory argumentCategory;

        static ConsoleArgument voxelSize;
    };

    template<typename PointT>
    ConsoleArgumentCategory VoxelGridDownsampler<PointT>::argumentCategory(
                "voxelgrid", "Voxel grid downsampling");

    template<typename PointT>
    ConsoleArgument VoxelGridDownsampler<PointT>::voxelSize = ConsoleArgument(
                "voxelgrid",
                "size",
                3.0f,
                "Leaf size of the voxel grid");
}

#endif // VoxelGridDownsampling_H
