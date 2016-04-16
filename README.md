# 3D Pose Estimation
This project features an object recognition pipeline to recognize and localize objects in a scene based on a variety of local features. Point clouds are given in the PCL format. By modularizing different techniques for

* Point cloud manipulation
* Feature description
* Keypoint extraction
* Transformation estimation
* Pose refinement,

the project aims at providing an experimentation platform that allows for fast evaluation of different 3d recognition methods.

## Requirements
* PCL >=1.8 built with CPP++11 support (requires VTK >=6.0.1)
* CMake >=3.1
* GCC >=4.9
* BOOST ~1.54
