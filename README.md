# 3D Pose Estimation
This project features an object recognition pipeline to recognize and localize objects in a scene based on a variety of local features. Point clouds are given in the PCD format. By modularizing different techniques for

* Point cloud manipulation
* Feature description
* Keypoint extraction
* Transformation estimation
* Pose refinement
* Hypothesis verification,

the project aims at providing an experimentation platform that allows for fast evaluation of different 3d recognition methods.

![PC Model Pose Estimation](https://github.com/aviate/pose-estimation/raw/documentation/pose-estimation.png)

![PC Model Pose Estimation](https://github.com/aviate/pose-estimation/raw/documentation/pose-estimation2.png)

## Requirements
* PCL >=1.8 built with C++11 support (requires VTK >=6.0.1, Eigen ~3.2.0)
* CMake >=3.1
* GCC >=4.9
* BOOST ~1.54
* NLopt

Additionally, for Unit Testing, CppUnit is required.

## Usage

`$ ./PoseEstimation (--folder foldername)|(model1.pcd model2.pcd ...) scene.pcd`

Find instances of a single model or a collection of models in a scene. Using the `--folder` CLI argument, a directory containing .pcd files representing the model candidates can be provided. Note that only the first .pcd file will be used as the input cloud for the optimization step to find good pose estimation parameters before the actual pipeline is executed on all models from the collection.

For each input model, every scene object is tested for descriptor correspondences. If the transformation estimation succeeds between an input model and a scene object, the system outputs the transformation matrix to transform the input model to the scene object, and the average of the correspondence distances ("uncertainty", between 0 and 1, the lower the more confident is the matching) between the two point clouds.

### Example
`$ ./PoseEstimation --folder objects book.pcd`

Load point clouds from directory "objects" (which contains 3 models) and find the best match with the scene point cloud.

The system prints out possible instances for each of those 3 models by showing the transformation matrices and uncertainties for each of the candidates.

#### Model 1
```
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching results for point cloud objects/book.pcd:
% Uncertainty: 0.343348
% Verified transformation(s):
    0.999768   0.00828556   -0.0198561     0.514141
 -0.00852746      0.99989   -0.0121292    0.0103949
   0.0197534    0.0122957     0.999729 -0.000965119
           0            0            0            1
Matching objects/book.pcd finished in 00:00:03.868636
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```
#### Model 2
```
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching results for point cloud objects/mustard_centered.pcd:
% Uncertainty: 0.597509
% Verified transformation(s):
-0.726818  0.282834  0.625891  0.522188
-0.674479 -0.121883 -0.728164 0.0735832
-0.129664 -0.951393  0.279353  0.586207
        0         0         0         1
Matching objects/mustard_centered.pcd finished in 00:00:00.669852
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```
#### Model 3
```
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Matching results for point cloud objects/book2.pcd:
% Uncertainty: 0.460474
% Verified transformation(s):
   0.999502 0.00724483 -0.0307228   0.519118
-0.00631464   0.999522  0.0302664  0.0159141
  0.0309274 -0.0300573    0.99907  0.0193727
          0          0          0          1
Matching objects/book2.pcd finished in 00:00:02.328845
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```
#### Result
Since Model 1 had the lowest uncertainty, it is presented as the best matching candidate:
```
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% The best matching point cloud is "objects/book.pcd" with an uncertainty of 0.343348.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
```

## Configuration

The system can be configured via its command line interface. Issue `$ PoseEstimation -h` to get a descriptive overview of all the available CLI parameters (similar to the table below).

A more comfortable way of setting the parameters is by providing a JSON file. The system stores such a file on each run and will read from the same file again to obtain the current settings. Configuration parameters are stored in a modular fashion following the system's ensemble of components, e.g. transformation estimators, feature descriptors, etc.

The following parameters are available:

| Parameter Name | Description | Type, Default value, Constraints |
| -------------- | ----------- | -------------------------------- |
| **Downsampler**	|     |     |
|     **uniformdown** | [Downsampling using uniform filtering](http://docs.pointclouds.org/trunk/classpcl_1_1_uniform_sampling.html)	|   |
|         `uniformdown_size`          | Sample size                                                | ([float] 1) constraints: (>= 1), (<= 10)	|
|     **voxelgrid** | [Downsampling using Voxel grid filtering](http://docs.pointclouds.org/trunk/classpcl_1_1_voxel_grid.html)	|   |
|         `voxelgrid_size`            | Leaf size of the voxel grid                                | ([float] 3) constraints: (>= 1), (<= 10)	|
| **KeypointExtractor**	|     |     |
|     **uniform** | [Keypoint extraction using Uniform Sampling](http://docs.pointclouds.org/trunk/classpcl_1_1_uniform_sampling.html)	|   |
|         `uniform_r`                 | Search radius for the uniform keypoint extraction          | ([float] 3.294)	|
|     **iss** | [Keypoint extraction using Intrinsic Shape Signatures (ISS)](http://docs.pointclouds.org/trunk/classpcl_1_1_i_s_s_keypoint3_d.html)	|   |
|         `iss_threads`               | Number of threads to use for ISS keypoint extraction       | ([int] 4)	|
|         `iss_nn`                    | Minimum number of neighbors to consider for ISS keypoint extraction | ([float] 3)	|
|         `iss_thresh21`              | ISS Threshold 21                                           | ([float] 0.975)	|
|         `iss_thresh32`              | ISS Threshold 32                                           | ([float] 0.975)	|
|         `iss_salient_r`             | Salient radius for ISS keypoint extraction                 | ([float] 3)	|
|         `iss_nonmax_r`              | Non maxima suppression radius for ISS keypoint extraction  | ([float] 8)	|
| **FeatureDescriptor**	|     |     |
|     **USC** | [Feature description using Unique Shape Context (USC)](http://docs.pointclouds.org/trunk/classpcl_1_1_unique_shape_context.html)	|   |
|         `USC_search_r`              | Search radius for finding neighbors                        | ([float] 25) constraints: (> pc_normal_nn)	|
|         `USC_min_r`                 | Minimum radius of the search sphere                        | ([float] 5)	|
|         `USC_density_r`             | Points within this radius are used to calculate the local point density | ([float] 10)	|
|         `USC_LRF_r`                 | Local Reference Frame (LRF) radius of USC descriptor       | ([float] 25)	|
|     **SI** | [Feature description using Spin Images (SI)](http://docs.pointclouds.org/trunk/classpcl_1_1_spin_image_estimation.html)	|   |
|         `SI_search_r`               | Search radius for finding neighbors                        | ([float] 5) constraints: (> pc_normal_nn)	|
|     **RSD** | [Feature description using Radius-based Surface Descriptor (RSD)](http://docs.pointclouds.org/trunk/classpcl_1_1_r_s_d_estimation.html)	|   |
|         `RSD_search_r`              | Search radius for finding neighbors                        | ([float] 5) constraints: (> pc_normal_nn)	|
|         `RSD_plane_r`               | Maximum radius, above which everything can be considered planar (should be 10-20 times RSD_search_r) | ([float] 30)	|
|         `RSD_save_hist`             | Whether to save histograms                                 | ([bool] 0)	|
|     **RIFT** | [Feature description using Rotation Invariant Feature Transform (RIFT)](http://docs.pointclouds.org/trunk/classpcl_1_1_r_i_f_t_estimation.html)	|   |
|         `RIFT_normal_nn`            | Search radius for finding neighbors for normal estimation  | ([float] 20)	|
|         `RIFT_gradient_r`           | Search radius for intensity gradient computation           | ([float] 50)	|
|         `RIFT_search_r`             | Search radius for finding neighbors                        | ([float] 50) constraints: (> pc_normal_nn)	|
|     **FPFH** | [Feature description using Fast Point Feature Histogram (FPFH)](http://docs.pointclouds.org/trunk/classpcl_1_1_f_p_f_h_estimation.html)	|   |
|         `FPFH_search_r`             | Search radius for finding neighbors                        | ([float] 33.41) constraints: (> pc_normal_nn)	|
|     **SHOT** | [Feature description using Signature of Histograms of OrienTations (SHOT)](http://docs.pointclouds.org/trunk/classpcl_1_1_s_h_o_t_color_estimation_o_m_p.html)	|   |
|         `SHOT_color`                | Consider color information                                 | ([bool] 0)	|
|         `SHOT_search_r`             | Search radius for finding neighbors                        | ([float] 15) constraints: (> pc_normal_nn)	|
|         `SHOT_LRF_r`                | Local Reference Frame (LRF) radius of SHOT descriptor      | ([float] 27.5)	|
| **FeatureMatcher**	|     |     |
|     **kdmatch** | [Feature matching using Kd-Trees](http://docs.pointclouds.org/trunk/classpcl_1_1search_1_1_kd_tree.html)	|   |
|         `kdmatch_thresh`          | Top percentage of correspondence distances that are considered | ([float] 0.424) constraints: (>= 0.1), (<= 1)	|
| **PoseRefiner**	    |     |     |
|     **icp** | [Pose refinement using Iterative Closest Point (ICP)](http://docs.pointclouds.org/trunk/classpcl_1_1_iterative_closest_point.html)	|   |
|         `icp_corrs_thresh`          | Maximum distance threshold between two correspondent points in source <-> target | ([float] 2)	|
|         `icp_iter`                  | Maximum number of iterations                               | ([int] 50)	|
|         `icp_trans_eps`             | Maximum allowable difference between two consecutive transformations | ([float] 1e-08)	|
|         `icp_fit_eps`               | Maximum allowed Euclidean error between two consecutive steps in the ICP loop | ([float] 1)	|
|     **ndt** | [Pose refinement using Normal Distributions Transform (NDT)](http://docs.pointclouds.org/trunk/classpcl_1_1_normal_distributions_transform.html)	|   |
|         `ndt_trans_eps`             | Minimum allowable difference between two consecutive transformations | ([float] 0.0001)	|
|         `ndt_step`                  | Maximum step size for More-Thuente line search             | ([float] 0.05)	|
|         `ndt_res`                   | Resolution of NDT grid structure (VoxelGridCovariance)     | ([float] 0.01)	|
|         `ndt_iter`                  | Maximum number of iterations                               | ([int] 2)	|
| **TransformationEstimator** |     |     |
|     **gc** | [Transformation estimation using Geometric Consistency (Correspondence Grouping)](http://docs.pointclouds.org/trunk/classpcl_1_1_geometric_consistency_grouping.html)	|   |
|         `gc_resolution`             | Consensus set resolution                                   | ([float] 3.6) constraints: (>= 1), (<= 10)	|
|         `gc_thresh`                 | Minimum cluster size                                       | ([float] 1.26) constraints: (>= 0.1), (<= 1)	|
|     **svd** | [Transformation estimation using Singular Value Decomposition (SVD)](http://docs.pointclouds.org/trunk/classpcl_1_1registration_1_1_transformation_estimation_s_v_d.html)	|   |
|     N/A | Not parameterizable	|   |
|     **ransac** | [Transformation estimation using prerejective RANSAC](http://docs.pointclouds.org/trunk/classpcl_1_1_sample_consensus_prerejective.html)	|   |
|         `ransac_iter`               | Maximum number of iterations                               | ([int] 50000)	|
|         `ransac_samples`            | Number of points to sample for generating/prerejecting a pose | ([int] 3)	|
|         `ransac_features`           | Number of nearest features to use                          | ([int] 5)	|
|         `ransac_sim_thresh`         | Polygonal edge length similarity threshold                 | ([float] 0.9)	|
|         `ransac_corr_thresh`        | Maximum correspondence distance for inlier consideration   | ([float] 2.5)	|
|         `ransac_hyp_thresh`         | Required inlier fraction for accepting a pose hypothesis   | ([float] 0.25)	|
|     **hough** | [Transformation estimation using Hough 3D Voting (Correspondence Grouping)](http://docs.pointclouds.org/trunk/classpcl_1_1_hough3_d_grouping.html)	|   |
|         `hough_bin_size`            | Size per bin into the Hough space                          | ([float] 5)	|
|         `hough_thresh`              | Minimum number of votes in the Hough space needed to infer the presence of a model instance into the scene cloud | ([float] 5)	|
| **LocalReferenceFrameEstimator** |     |     |
|     **BOARD** | [Local Reference Frame Estimation using BOrder Aware Repeatable Directions (BOARD)](http://docs.pointclouds.org/trunk/classpcl_1_1_b_o_a_r_d_local_reference_frame_estimation.html)	|   |
|         `BOARD_holes`               | Search and account for holes in the margin of the support  | ([bool] 1)	|
|         `BOARD_search_r`            | Search radius of BOARD LRF estimation                      | ([float] 10)	|
| **HypothesisVerifier** |     |     |
|     **hv** | [Global Hypothesis Verification](http://docs.pointclouds.org/trunk/classpcl_1_1_global_hypotheses_verification.html)	|   |
|         `hv_inlier_thresh`          | Inlier threshold                                           | ([float] 0.005) constraints: (>= 0), (<= 1)	|
|         `hv_occlusion_thresh`       | Occlusion threshold                                        | ([float] 0.01) constraints: (>= 0), (<= 1)	|
|         `hv_regularizer`            | Regularizer value                                          | ([float] 3)	|
|         `hv_clutter_r`              | Clutter radius                                             | ([float] 0.03) constraints: (>= 0), (<= 1)	|
|         `hv_clutter_regularizer`    | Clutter regularizer                                        | ([float] 2)	|
|         `hv_clutter`                | Whether to perform clutter detection                       | ([bool] 1)	|
|         `hv_normal_r`               | Search radius for normal estimation                        | ([float] 0.05) constraints: (>= 0), (<= 1)	|
| **Miscellaneous**     |     |     |
|     **opt** | [Non-Linear Optimization for Pipeline Module Parameters](http://ab-initio.mit.edu/wiki/index.php/NLopt)	|   |
|         `opt_skip_descriptor`       | Skip optimization of feature description parameters        | ([bool] 0)	|
|         `opt_skip_downsampler`      | Skip optimization of downsampling parameters               | ([bool] 0)	|
|         `opt_skip_feature_matcher`  | Skip optimization of feature matching parameters           | ([bool] 0)	|
|         `opt_skip_keypoint_extractor` | Skip optimization of keypoint extraction parameters        | ([bool] 0)	|
|         `opt_skip_transformation_estimator` | Skip optimization of transformation estimation parameters  | ([bool] 0)	|
|         `opt_skip_pose_refiner`     | Skip optimization of pose refinement parameters            | ([bool] 0)	|
|         `opt_skip_hypothesis_verifier` | Skip optimization of hypothesis verification parameters    | ([bool] 1)	|
|         `opt_skip_misc`             | Skip optimization of miscellaneous parameters              | ([bool] 0)	|
|         `opt_xdelta`                | Minimum allowed changed of all parameter values (stopping criterion) | ([float] 0.5)	|
|         `opt_iterations`            | Maximum number of iterations (stopping criterion)          | ([int] 40)	|
|         `opt_skip_init`             | Skip initialization and begin with default parameter settings | ([bool] 1)	|
|         `opt_alpha`                 | Relative parameter value during initialization in the corresponding value range | ([float] 0.2) constraints: (>= 0), (<= 1)	|
|         `opt_enabled`               | Whether to optimize pipeline module parameters             | ([bool] 0)	|
|     **config** | Configuration of modules to use for pose estimation pipeline	|   |
| 		`config_descriptor`         | Feature descriptor module                                  | ([FPFH/RIFT/RSD/SHOT/SI/USC] FPFH)	|
| 		`config_transformation_estimator` | Transformation estimator module                            | ([gc/hough/ransac/svd] gc)	|
| 		`config_downsampler`        | Downsampler module                                         | ([uniformdown/voxelgrid] uniformdown)	|
| 		`config_feature_matcher`    | Feature matcher module                                     | ([kdmatch] kdmatch)	|
| 		`config_keypoint_extractor` | Keypoint extractor module                                  | ([iss/uniform] uniform)	|
| 		`config_poserefiner`        | Iterative pose refinement module                           | ([icp/ndt] icp)	|
|     **pc** | Point Cloud computations	|   |
|         `pc_normal_nn`              | Search radius of nearest neighbor normal estimation        | ([float] 9.725) constraints: (>= 3), (<= 20)	|
|     **pipeline** | Pose estimation pipeline	|   |
|         `pipeline_downsampling`     | Whether to use the downsampling module while processing the pipeline | ([bool] 1)	|
|         `pipeline_keypoint`         | Whether to use the keypoint extraction module while processing the pipeline | ([bool] 1)	|
|         `pipeline_pose_refine`      | Whether to use the pose refinement module while processing the pipeline | ([bool] 0)	|
|         `pipeline_hyp_ver`          | Whether to use the hypothesis verification module while processing the pipeline | ([bool] 1)	|
|         `pipeline_max_descs`        | Maximum allowable number of descriptors per cloud to be calculated | ([int] 300000)	|

The [Point Cloud Library (PCL)](http://pointclouds.org/) provides most point-cloud related algorithms. All of the involved parameters are resolution-independent by multiplying applicable parameters with the computed point cloud resolution. The resolution is defined as the mean distance of two closest points in the cloud.

[NLopt](http://ab-initio.mit.edu/wiki/index.php/NLopt) is used for the non-linear parameter optimization.
