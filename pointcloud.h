#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "logger.h"
#include "types.h"
#include "parameter.h"

namespace PoseEstimation
{
    template<typename PointT>
    class PC
    {
    public:
        PC(const typename pcl::PointCloud<PointT>::Ptr cloud = typename pcl::PointCloud<PointT>::Ptr(new typename pcl::PointCloud<PointT>))
        {
            _resolution = -1;
            _cloud = cloud;
            _normals = PclNormalCloud::Ptr(new PclNormalCloud);
        }

        /**
         * @brief Loads a point cloud from an ASCII PCD file.
         * @param filename The file name.
         * @param pc The target point cloud.
         * @return True, if the point cloud has been loaded successfully.
         */
        static bool loadFromFile(const std::string &filename, PC<PointT> &pc)
        {
            Logger::debug(boost::format("Loading cloud from \"%s\".") % filename);
            return pcl::io::loadPCDFile(filename, *(pc._cloud)) >= 0;
        }

        /**
         * @brief Save the point cloud as an ASCII PCD file.
         * @param filename The target file name.
         * @return True, if the point cloud has been stored successfully.
         */
        bool saveToFile(const std::string &filename) const
        {
            Logger::debug(boost::format("Saving cloud to \"%s\".") % filename);
            return pcl::io::savePCDFileASCII(filename, *_cloud) >= 0;
        }

        /**
         * @brief Calculates the cloud granularity.
         * @details Computes the average distance between two nearest points in the cloud.
         * @return Cloud resolution.
         */
        double resolution()
        {
            if (_resolution <= 0)
                _compute_resolution();
            //Debugger::debug(boost::format("Resolution: %d") % _resolution);
            return _resolution;
        }

        /**
         * @brief Returns the number of points in the point cloud.
         * @return Number of points in the point cloud.
         */
        size_t size() const
        {
            if (_cloud.get())
                return _cloud->size();
            return 0;
        }

        /**
         * @brief Checks whether there are points in the point cloud.
         * @return True, if the point cloud does not contain any points.
         */
        bool empty() const
        {
            return !_cloud.get() || _cloud->points.empty();
        }

        /**
         * @brief Computes the centroid of the point cloud.
         * @return The centroid.
         */
        PointT centroid() const
        {
            pcl::CentroidPoint<PointT> centroid;
            for (int i = 0; i < _cloud->points.size(); ++i)
                centroid.add(_cloud->points[i]);

            PointT c;
            centroid.get(c);
            return c;
        }

        /**
         * @brief Returns the PCL point cloud.
         * @return The PCL point cloud.
         */
        typename pcl::PointCloud<PointT>::Ptr cloud() const
        {
            return _cloud;
        }

        /**
         * @brief Returns the point type of the point cloud.
         * @return The point type.
         */
        std::type_info type() const
        {
            return typeid(PointT);
        }

        /**
         * @brief Centers a point cloud.
         * @details Uses the centroid of a point cloud to align the cloud at the origin
         * of the xyz coordinate system.
         *
         * @param cloud The input cloud which will be altered by this function.
         */
        void center()
        {
            PointT center = centroid();
            // center points
            translate(-center.x, -center.y, -center.z);
        }

        /**
         * @brief Translates point cloud by vector.
         * @param x X coordinate of translation vector.
         * @param y Y coordinate of translation vector.
         * @param z Z coordinate of translation vector.
         */
        void translate(float x, float y, float z)
        {
            for (int i = 0; i < _cloud->points.size(); ++i)
            {
                _cloud->points[i].x += x;
                _cloud->points[i].y += y;
                _cloud->points[i].z += z;
            }
        }

        /**
         * @brief Transforms the point cloud by the transformation matrix.
         * @param transformation The 4x4 transformation matrix.
         */
        void transform(const Eigen::Matrix4f &transformation)
        {
            pcl::transformPointCloud(*_cloud, *_cloud, transformation);
        }

        /**
         * @brief Estimates normals for the point cloud.
         * @return PCL point cloud containing the normals.
         */
        PclNormalCloud::Ptr normals()
        {
            if (_normals->points.empty())
            {
                pcl::NormalEstimationOMP<PointT, NormalType> ne;
                typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
                ne.setSearchMethod(tree);

                ne.setRadiusSearch(resolution() * normalEstimationRadius.value<float>());
                ne.setInputCloud(_cloud);

                _normals = PclNormalCloud::Ptr(new PclNormalCloud);
                ne.compute(*_normals);

                //TODO Faster normal computation possible using Integral Images for organized point clouds...
                // http://pointclouds.org/documentation/tutorials/normal_estimation_using_integral_images.php

                //XXX smooth normals?
            }

            return _normals;
        }

        /**
         * @brief Estimates normals at the given target points for this point cloud.
         * @param targetPoints Points at which the normals will be estimated.
         * @return PCL point cloud containing the normals.
         */
        PclNormalCloud::Ptr normals(const PclPointCloud::Ptr &targetPoints)
        {
            PclNormalCloud::Ptr ns = normals();
            PclNormalCloud::Ptr targetNormals(new PclNormalCloud);

            // accelerate search for equal points using an Kd tree
            typename pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
            kdtree->setInputCloud(_cloud);
            size_t idx = 0;
            std::vector<int> pointIdxNKNSearch(1);
            std::vector<float> pointNKNSquaredDistance(1);
            for (PointT &targetPoint : targetPoints->points)
            {
                if (kdtree->nearestKSearchT(targetPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0)
                {
                    Logger::warning(boost::format("Normal estimation failed for target point %i. " \
                                                  "No matching point could be found in the point cloud.") % idx);
                    continue;
                }

                targetNormals->push_back((*ns)[pointIdxNKNSearch[0]]);

                ++idx;
            }

            return targetNormals;
        }

        /**
         * @brief Resets normals and cloud resolution to be updated on next access.
         */
        void update()
        {
            _normals->clear();
            _resolution = 0;
        }

        static ParameterCategory argumentCategory;
        static Parameter normalEstimationRadius;

    private:
        typename pcl::PointCloud<PointT>::Ptr _cloud;
        PclNormalCloud::Ptr _normals;
        double _resolution;

        void _compute_resolution()
        {
            _resolution = 0.0;
            int n_points = 0;
            int nres;
            std::vector<int> indices(2);
            std::vector<float> sqr_distances(2);
            pcl::search::KdTree<PointT> tree;
            tree.setInputCloud(_cloud);

            for (size_t i = 0; i < _cloud->size(); ++i)
            {
                if (!pcl_isfinite((*_cloud)[i].x))
                    continue;

                // Considering the second neighbor since the first is the point itself.
                nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
                if (nres == 2)
                {
                    _resolution += sqrt(sqr_distances[1]);
                    ++n_points;
                }
            }
            if (n_points != 0)
            {
                _resolution /= n_points;
            }
        }
    };

    typedef PC<PointType> PointCloud;
    typedef PC<NormalType> NormalCloud;

    template<typename PointT>
    ParameterCategory PC<PointT>::argumentCategory(
            "pc", "Point Cloud computations");

    template<typename PointT>
    Parameter PC<PointT>::normalEstimationRadius = Parameter(
            "pc", "normal_nn", (float)20.0f, "Search radius of nearest neighbor normal estimation");
}

#endif // POINTCLOUD_H
