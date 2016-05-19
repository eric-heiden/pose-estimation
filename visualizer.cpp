#include "visualizer.h"

#include "logger.h"

using namespace PoseEstimation;

VisualizerObject::VisualizerObject(std::string id) : _id(id)
{
}

void VisualizerObject::setPointSize(double size)
{
    if (Visualizer::_enabled)
        Visualizer::_instance.viewer()->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                size, _id);
}

bool &Visualizer::enabled()
{
    return _enabled;
}

VisualizerObject Visualizer::visualize(const PointCloud &pc)
{
    if (!_enabled)
        return VisualizerObject();

    Logger::log(boost::format("visualizing %d points") % pc.cloud()->points.size());
    VisualizerObject ob(object_name());
    _instance.viewer()->addPointCloud(pc.cloud(), ob._id);
    return ob;
}

VisualizerObject Visualizer::visualize(const PointCloud &pc, const Color &c)
{
    if (!_enabled)
        return VisualizerObject();

    VisualizerObject ob(object_name());
    if (c.r >= 0 && c.g >= 0 && c.b >= 0)
    {
        pcl::visualization::PointCloudColorHandlerCustom<PointType> color_handler(
                    pc.cloud(), 255.0*c.r, 255.0*c.g, 255.0*c.b);
        _instance.viewer()->addPointCloud(pc.cloud(), color_handler, ob._id);
    }
    else
    {
        _instance.viewer()->addPointCloud(pc.cloud(), ob._id);
    }

    return ob;
}

VisualizerObject Visualizer::visualize(const PointType &p1, const PointType &p2, const Color &c)
{
    if (!_enabled)
        return VisualizerObject();

    VisualizerObject ob(object_name());
    _instance.viewer()->addLine<PointType, PointType>(p1, p2, 255.0*c.r, 255.0*c.g, 255.0*c.b, ob._id);
    return ob;
}

void Visualizer::render()
{
    while (_enabled && !_instance.viewer()->wasStopped())
    {
        _instance.viewer()->spinOnce();
    }
}

Visualizer::Visualizer(const std::string &title) : _title(title)
{
}

pcl::visualization::PCLVisualizer *Visualizer::viewer()
{
    if (!_viewer)
    {
        _viewer = new pcl::visualization::PCLVisualizer(
                    pcl::visualization::PCLVisualizer(_title));
        _viewer->setBackgroundColor(1, 1, 1);
    }

    return _viewer;
}

std::string Visualizer::object_name()
{
    std::stringstream ss;
    ss << "object " << ++_obj_counter;
    return ss.str();
}

Visualizer Visualizer::_instance = Visualizer();
unsigned int Visualizer::_obj_counter = 0;
bool Visualizer::_enabled = true;
