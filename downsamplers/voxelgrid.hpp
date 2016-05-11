#pragma once

#include <pcl/filters/approximate_voxel_grid.h>

#include "../types.h"
#include "../parameter.h"
#include "../pointcloud.h"
#include "../downsampling.hpp"

namespace PoseEstimation
{
    template<typename PointT>
    class VoxelGridDownsampler : public Downsampler<PointT>
    {
    public:
        VoxelGridDownsampler() : Downsampler<PointT>()
        {
        }

        virtual void downsample(PC<PointT> &pc, PC<PointT> &downsampled) const
        {
            pcl::ApproximateVoxelGrid<PointT> approximate_voxel_filter;
            float sample_size = voxelSize.value<float>() * pc.resolution();
            approximate_voxel_filter.setLeafSize(sample_size, sample_size, sample_size);
            approximate_voxel_filter.setInputCloud(pc.cloud());
            approximate_voxel_filter.filter(*(downsampled.cloud()));
            downsampled.update();
        }

        static ParameterCategory argumentCategory;
        PARAMETER_CATEGORY_GETTER(argumentCategory)

        static Parameter voxelSize;
    };

    template<typename PointT>
    ParameterCategory VoxelGridDownsampler<PointT>::argumentCategory(
                "voxelgrid", "Voxel grid downsampling",
                PipelineModuleType::Downsampler);

    template<typename PointT>
    Parameter VoxelGridDownsampler<PointT>::voxelSize = Parameter(
                "voxelgrid",
                "size",
                3.0f,
                "Leaf size of the voxel grid");
}
