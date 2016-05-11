#pragma once

#include <pcl/visualization/pcl_visualizer.h>

#include "types.h"
#include "color.hpp"
#include "pointcloud.h"

namespace PoseEstimation
{
    const std::string VISUALIZATION_NAME = "Pose Estimation";
    class Visualizer;
    class VisualizerObject
    {
        friend class Visualizer;
    public:
        void setPointSize(double size);

    private:
        VisualizerObject(std::string id = "N/A");
        std::string _id;
    };

    class Visualizer
    {
        friend class VisualizerObject;
    public:
        static bool &enabled();

        static VisualizerObject visualize(const PointCloud &pc);
        static VisualizerObject visualize(const PointCloud &pc, const PoseEstimation::Color &c);

        static VisualizerObject visualize(const PointType &p1, const PointType &p2, const PoseEstimation::Color &c);
        static void render();

    private:
        Visualizer(const std::string &title = VISUALIZATION_NAME);
        const std::string _title;
        static Visualizer _instance;
        std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer;
        std::shared_ptr<pcl::visualization::PCLVisualizer> viewer();
        static unsigned int _obj_counter;

        static bool _enabled;

        static std::string object_name();
    };
}
