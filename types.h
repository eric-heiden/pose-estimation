#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace PoseEstimation
{
    /*******************************
     *  Frequently used PCL types  *
     *******************************/

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointNormal NormalType;
    typedef pcl::ReferenceFrame RFType;
    typedef pcl::PointCloud<PointType> PclPointCloud;
    typedef pcl::PointCloud<NormalType> PclNormalCloud;
}

#endif // TYPES_H
