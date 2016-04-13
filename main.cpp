#include <iostream>

#include <pcl/console/parse.h>

#include "logger.h"
#include "consoleargument.h"
#include "pointcloud.h"
#include "visualizer.h"
#include "pipeline.hpp"

using namespace PoseEstimation;

void showHelp(char *appname)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*                             Pose Estimation                             *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "Usage: " << *appname << " source_filename.pcd target_filename.pcd" << std::endl << std::endl;
    ConsoleArgument::displayAll();
}

int main(int argc, char **argv)
{
    ConsoleArgument::parseAll(argc, argv);

    if (pcl::console::find_switch(argc, argv, "-h"))
        showHelp(argv[0]);

    // source & target pcl filenames
    std::vector<int> filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
    if (filenames.size() != 2)
    {
        Logger::error("Filenames missing.");
        showHelp(argv[0]);
        std::exit(EXIT_FAILURE);
    }

    std::string source_filename = argv[filenames[0]];
    std::string target_filename = argv[filenames[1]];
    PointCloud source, target;
    if (!PointCloud::loadFromFile(source_filename, source))
    {
        Logger::error(boost::format("Couldn't load source cloud from \"%s\".") % source_filename);
        std::exit(EXIT_FAILURE);
    }
    if (!PointCloud::loadFromFile(target_filename, target))
    {
        Logger::error(boost::format("Couldn't load target cloud from \"%s\".") % target_filename);
        std::exit(EXIT_FAILURE);
    }

    // move target cloud to the right to visualize source & target side by side
    target.translate(1, 0, 0);

    // actual pose estimation pipeline
    Pipeline::process(source, target);

    Visualizer::visualize(source);
    Visualizer::visualize(target);

    Visualizer::render();

    return 0;
}

