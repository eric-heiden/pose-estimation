#pragma once

//TODO to be removed...

#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/uniform_sampling.h>

#include <pcl/common/centroid.h>

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "types.h"

namespace PoseEstimation
{
    /**********************************************************
     *     Useful helper methods (unused at this moment)      *
     **********************************************************/

    const double PI = 3.14159265358979323846264338327950288419716939937510582;
    const double DEG2RAD = PI / 180.0;
    const double RAD2DEG = 180.0 / PI;

    /**
     * @brief Finds the minimum element in an array.
     * @details Returns the index of the minimum element in an array. -1 if array is empty / null.
     *
     * @param array The input array.
     * @return The index of the minimum element.
     */
    template<class T>
    int min_element(const T *array)
    {
        if (!array)
            return -1;
        int length = sizeof(array) / sizeof(T);
        if (length <= 0)
            return -1;
        int m = 0;
        for (int i = 1; i < length; ++i)
        {
            if (array[i] < array[m])
                m = i;
        }
        return m;
    }

    /**
     * @brief Calculates the squared distance between two points.
     * @details Calculates the squared Euclidean distance between the two points.
     *
     * @param a First point with xyz-coordinates.
     * @param b Second point with xyz-coordinates.
     *
     * @return Squared distance between a and b.
     */
    float sqr_distance(const PointType &a, const PointType &b)
    {
        return pow(a.x-b.x, 2) + pow(a.y-b.y, 2) + pow(a.z-b.z, 2);
    }

    /**
     * @brief Calculates the squared distance between two points.
     * @details Calculates the squared Euclidean distance between the two points.
     *
     * @param a First point with xyz-coordinates.
     * @param b Second point with xyz-coordinates.
     *
     * @return Squared distance between a and b.
     */
    float sqr_distance(const Eigen::Vector3f &a, const Eigen::Vector3f &b)
    {
        return pow(a[0]-b[0], 2) + pow(a[1]-b[1], 2) + pow(a[2]-b[2], 2);
    }

    /**
     * @brief Converts a PCL point to an Eigen vector.
     * @details Converts a PCL point to an Eigen 3d vector by using the xyz-coordinates.
     *
     * @param point The PCL point.
     * @return The Eigen vector.
     */
    Eigen::Vector3f point2vec(const PointType &point)
    {
        return Eigen::Vector3f(point.x, point.y, point.z);
    }

    /**
     * @brief Converts an Eigen vector to a PCL point.
     * @details Converts an Eigen 3d vector to a PCL point by using the xyz-coordinates.
     *
     * @param point The PCL point.
     * @return The Eigen vector.
     */
    PointType vec2point(const Eigen::Vector3f &point)
    {
        PointType p;
        p.x = point[0];
        p.y = point[1];
        p.z = point[2];
        return p;
    }

    /**
     * @brief Generates a skew-symmetric matrix from a vector.
     * @details The skew-symmetric matrix S from a vector u allows to calculate the
     * cross-product u x v by using only the matrix multiplication S * v.
     *
     * @param v The input vector.
     * @return The skew-symmetric matrix of v.
     */
    template <typename T>
    inline Eigen::Matrix<T, 3, 3> makeSkewSymmetric(const Eigen::Matrix<T, 3, 1> &v)
    {
        Eigen::Matrix<T, 3, 3> out;
        out <<	   0, -v[2],  v[1],
                v[2],     0, -v[0],
               -v[1],  v[0],     0;

        return out;
    }
}
