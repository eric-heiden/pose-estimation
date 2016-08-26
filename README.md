# 3D Pose Estimation
This project features an object recognition pipeline to recognize and localize objects in a scene based on a variety of local features. Point clouds are given in the PCL format. By modularizing different techniques for

* Point cloud manipulation
* Feature description
* Keypoint extraction
* Transformation estimation
* Pose refinement
* Hypothesis verification,

the project aims at providing an experimentation platform that allows for fast evaluation of different 3d recognition methods.

![PC Model Pose Estimation](https://github.com/aviate/pose-estimation/raw/documentation/pose-estimation.png)

## Requirements
* PCL >=1.8 built with C++11 support (requires VTK >=6.0.1)
* CMake >=3.1
* GCC >=4.9
* BOOST ~1.54
* NLopt

Additionally, for Unit Testing, CppUnit is required.

## Usage

Find instances of a single model or a collection of models in a scene. Using the --folder CLI argument, a directory of .pcd files containing the model candidates can be provided.

The system will first identify clusters in the scene point cloud that serve as potential matching candidates for the provided models. For each input model, every scene object is tested for descriptor correspondences. If the transformation estimation succeeds between an input model and a scene object, the system outputs the transformation matrix to transform the input model to the scene object, and the average of the correspondence distances ("uncertainty", between 0 and 1, the lower the more confident is the matching) between the two point clouds.

## Configuration

The system can be configured via its CLI. Issue `$ PoseEstimation -h` to get an descriptive overview of all the available CLI parameters.

A more comfortable way of setting the parameters is by providing a JSON file. The system stores such a file on each run and will read from the same file again to obtain the current settings. Configuration parameters are stored in a modular fashion following the system's ensemble of components, e.g. transformation estimators, feature descriptors, etc.
