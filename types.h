#ifndef TYPES_H
#define TYPES_H

#include <pcl/point_types.h>

namespace PoseEstimation
{
    /*******************************
     *  Frequently used PCL types  *
     *******************************/

    typedef pcl::PointXYZRGB PointType;
    typedef pcl::PointNormal NormalType;
    typedef pcl::ReferenceFrame RFType;
}

#endif // TYPES_H
