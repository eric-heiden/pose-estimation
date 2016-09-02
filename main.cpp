#include <iostream>

#include <boost/filesystem.hpp>
#include <pcl/console/parse.h>

#include "logger.h"
#include "parameter.h"
#include "pointcloud.h"
#include "visualizer.h"
#include "pipeline.hpp"
#include "configuration.hpp"
#include "optimization/optimizer.hpp"

namespace fs = ::boost::filesystem;
using namespace PoseEstimation;

void showHelp(char *appname)
{
    std::cout << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "*           Point Cloud Model Matching and Pose Estimation                *" << std::endl;
    std::cout << "*                                                                         *" << std::endl;
    std::cout << "***************************************************************************" << std::endl;
    std::cout << "Usage: " << *appname << " (--folder foldername)|(model1.pcd model2.pcd ...) scene.pcd"
              << std::endl << std::endl;
    Parameter::displayAll();
}

void loadPointCloud(std::string filename, PointCloud &cloud)
{
    if (!PointCloud::loadFromFile(filename, cloud))
    {
        Logger::error(boost::format("Couldn't load point cloud from \"%s\".") % filename);
        std::exit(EXIT_FAILURE);
    }
}

int main(int argc, char **argv)
{
    Logger::log(boost::format("Eigen version %1%.%2%.%3%") %
                EIGEN_WORLD_VERSION % EIGEN_MAJOR_VERSION % EIGEN_MINOR_VERSION);
    // disable PCL warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    // ensure static parameters are properly initialized
    Optimizer::argumentCategory.parameters();
    Configuration::argumentCategory.parameters();

    if (pcl::console::find_switch(argc, argv, "-h"))
        showHelp(argv[0]);

    //
    // Loading input point clouds
    //

    bool folder_mode = pcl::console::find_switch(argc, argv, "--folder");

    // Model & scene filenames
    std::vector<int> filename_idxs = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if (filename_idxs.empty() || !folder_mode && filename_idxs.size() < 2)
    {
        Logger::error("Filenames missing.");
        showHelp(argv[0]);
        std::exit(EXIT_FAILURE);
    }

    std::vector<std::string> model_filenames;
    std::string scene_filename = argv[filename_idxs.back()];
    for (auto &idx : filename_idxs)
    {
        // last filename is the scene point cloud
        if (idx != filename_idxs.back())
            model_filenames.push_back(argv[idx]);
    }

    if (folder_mode)
    {
        std::string foldername;
        pcl::console::parse_argument(argc, argv, "--folder", foldername);
        Logger::log(boost::format("Loading model point clouds from folder \"%s\"...") % foldername);

        if (!fs::exists(foldername) || !fs::is_directory(foldername))
        {
            Logger::error("Folder does not exist!");
            std::exit(EXIT_FAILURE);
        }

        fs::recursive_directory_iterator it(foldername);
        fs::recursive_directory_iterator endit;

        while (it != endit)
        {
            if (fs::is_regular_file(*it)
                && it->path().extension() == ".pcd")
            {
                model_filenames.push_back(it->path().string());
                Logger::debug(boost::format("Found point cloud \"%s\".") % model_filenames.back());
            }
            ++it;
        }

        if (model_filenames.empty())
        {
            Logger::error("Could not find any point clouds! Make sure to store .pcd files in the given directory.");
            std::exit(EXIT_FAILURE);
        }
    }

    // load scene point cloud
    PointCloud target, source;
    loadPointCloud(scene_filename, target);

    // move target cloud to the right to visualize source & target side by side
    target.translate(0.5, 0.0, 0.0); //XXX this will alter the resulting transformation matrix!

    // load parameters from CLI and JSON configuration file
    Parameter::parseAll(argc, argv);
    Parameter::loadAll("configuration.json");

    //
    // Optimize parameters
    //

    Configuration config;

    // for optimization, only the first given point cloud is used as source
    loadPointCloud(model_filenames.front(), source);
    Optimizer opt(source, target);

//    Parameter::set("opt_enabled", true);
//    Parameter::set("opt_skip_descriptor", true);
//    Parameter::set("opt_skip_downsampler", true);
//    Parameter::set("opt_skip_feature_matcher", true);
//    Parameter::set("opt_skip_keypoint_extractor", true);
//    Parameter::set("opt_skip_transformation_estimator", true);
//    Parameter::set("opt_skip_pose_refiner", true);
//    Parameter::set("opt_skip_misc", true);
//    Parameter::set("opt_skip_hypothesis_verifier", false);
//    Parameter::set("pipeline_skip_te", false);

    Visualizer::setEnabled(false);

    Logger::tic("Optimization");
    OptimizationResult res = opt.optimize(config);
    Logger::toc("Optimization");

    // save optimal configuration
    Parameter::saveAll("optimal_configuration.json");

    for (auto &assignment : res.bestAssignment)
    {
        Parameter::get(assignment.first)->setNumericalValue(assignment.second);
        Logger::debug(boost::format("Setting %s = %d") % assignment.first % assignment.second);
    }

    Visualizer::setEnabled(true);

    //
    // Object matching and pose estimation
    //

    float minUncertainty = std::numeric_limits<float>::max();
    std::string bestModel = "";
    for (std::string &model_filename : model_filenames)
    {
        Logger::tic("Matching " + model_filename);
        loadPointCloud(model_filename, source);
        PipelineStats result = config.run(source, target);
        Logger::log(std::string(50, '%'));

        if (result.verifiedTransformationInstances.empty())
        {
            Logger::log(boost::format("%% Point cloud %s did not match the scene.") % model_filename);
        }
        else
        {
            Logger::log(boost::format("%% Matching results for point cloud %s:") % model_filename);
            Logger::log(boost::format("%% Uncertainty: %.6f") % result.averageCorrespondenceDistance);
            Logger::log("% Verified transformation(s):");
            for (auto &transformation : result.verifiedTransformationInstances)
                Logger::log(boost::format("%1%") % transformation);
            if (result.averageCorrespondenceDistance < minUncertainty)
            {
                minUncertainty = result.averageCorrespondenceDistance;
                bestModel = model_filename;
            }
        }

        Logger::toc("Matching " + model_filename);
        Logger::log(std::string(50, '%'));
    }

    if (folder_mode)
    {
        Logger::log(std::string(100, '%'));
        if (bestModel.empty())
            Logger::error("% None of the provided models matched the target point cloud.");
        else
            Logger::log(boost::format("%% The best matching point cloud is \"%s\" with an uncertainty of %.6f.")
                        % bestModel % minUncertainty);
        Logger::log(std::string(100, '%'));
    }

    //
    // Visualization
    //

    Visualizer::visualize(source);
    Visualizer::visualize(target);

    Visualizer::render();

    return 0;
}
