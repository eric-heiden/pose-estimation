#include "visualizer.h"

#include "logger.h"

using namespace PoseEstimation;

VisualizerObject::VisualizerObject(std::string id) : _id(id)
{
}

void VisualizerObject::setPointSize(double size)
{
    Visualizer::_instance._viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                size, _id);
}

Visualizer::~Visualizer()
{
    delete _viewer;
}

VisualizerObject Visualizer::visualize(const PointCloud &pc)
{
    Logger::log(boost::format("visualizing %d points") % pc.cloud()->points.size());
    VisualizerObject ob(object_name());
    _instance._viewer->addPointCloud(pc.cloud(), ob._id);
    return ob;
}

VisualizerObject Visualizer::visualize(const PointCloud &pc, const Color &c)
{
    VisualizerObject ob(object_name());
    if (c.r >= 0 && c.g >= 0 && c.b >= 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(
                    pc.cloud(), 255.0*c.r, 255.0*c.g, 255.0*c.b);
        _instance._viewer->addPointCloud(pc.cloud(), color_handler, ob._id);
    }
    else
    {
        _instance._viewer->addPointCloud(pc.cloud(), ob._id);
    }
    return ob;
}

VisualizerObject Visualizer::visualize(const PointType &p1, const PointType &p2, const Color &c)
{
    VisualizerObject ob(object_name());
    _instance._viewer->addLine<PointType, PointType>(p1, p2, 255.0*c.r, 255.0*c.g, 255.0*c.b, ob._id);
    return ob;
}

void Visualizer::render()
{
    while (!_instance._viewer->wasStopped())
    {
        _instance._viewer->spinOnce();
    }
}

Visualizer::Visualizer(const std::string &title)
{
    _viewer = new pcl::visualization::PCLVisualizer(title);
    _viewer->setBackgroundColor(1, 1, 1);
}

std::string Visualizer::object_name()
{
    std::stringstream ss;
    ss << "object " << ++_obj_counter;
    return ss.str();
}

Visualizer Visualizer::_instance = Visualizer();
unsigned int Visualizer::_obj_counter = 0;
