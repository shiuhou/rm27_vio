// Boundary adapter derived from pinned SchurVINS svo_ros/src/svo_factory.cpp.
// Parameter defaults/option mapping are preserved; ROS access is replaced by YAML.
#include <svo/svo.h>
#include <svo/common/imu_calibration.h>
#include <svo/frame_handler_mono.h>
#include <svo/frame_handler_stereo.h>
#include <svo/imu_handler.h>
#include <yaml-cpp/yaml.h>
namespace svo { namespace headless {
template<class T> T param(const YAML::Node& p, const std::string& key, const T& fallback, bool = false) {
  return p[key] ? p[key].as<T>() : fallback;
}
BaseOptions loadBaseOptions(const YAML::Node& pnh, bool forward_default)
{
  BaseOptions o;
  o.max_n_kfs = param<int>(pnh, "max_n_kfs", 5);
  o.use_imu = param<bool>(pnh, "use_imu", false);
  o.trace_dir = param<std::string>(pnh, "trace_dir", std::string("/tmp"));
  o.quality_min_fts = param<int>(pnh, "quality_min_fts", 50);
  o.quality_max_fts_drop = param<int>(pnh, "quality_max_drop_fts", 40);
  o.relocalization_max_trials = param<int>(pnh, "relocalization_max_trials", 50);
  o.poseoptim_prior_lambda = param<double>(pnh, "poseoptim_prior_lambda", 0.0);
  o.poseoptim_using_unit_sphere = param<bool>(pnh, "poseoptim_using_unit_sphere", false);
  o.img_align_prior_lambda_rot = param<double>(pnh, "img_align_prior_lambda_rot", 0.0);
  o.img_align_prior_lambda_trans = param<double>(pnh, "img_align_prior_lambda_trans", 0.0);
  o.structure_optimization_max_pts = param<int>(pnh, "structure_optimization_max_pts", 20);
  o.init_map_scale = param<double>(pnh, "map_scale", 1.0);
  std::string default_kf_criterion = forward_default ? "FORWARD" : "DOWNLOOKING";
  if(param<std::string>(pnh, "kfselect_criterion", default_kf_criterion) == "FORWARD")
    o.kfselect_criterion = KeyframeCriterion::FORWARD;
  else
    o.kfselect_criterion = KeyframeCriterion::DOWNLOOKING;
  o.kfselect_min_dist = param<double>(pnh, "kfselect_min_dist", 0.12);
  o.kfselect_numkfs_upper_thresh = param<int>(pnh, "kfselect_numkfs_upper_thresh", 120);
  o.kfselect_numkfs_lower_thresh = param<double>(pnh, "kfselect_numkfs_lower_thresh", 70);
  o.kfselect_min_dist_metric = param<double>(pnh, "kfselect_min_dist_metric", 0.01);
  o.kfselect_min_angle = param<double>(pnh, "kfselect_min_angle", 20);
  o.kfselect_min_disparity = param<double>(pnh, "kfselect_min_disparity", 40);
  o.kfselect_min_num_frames_between_kfs = param<int>(pnh, "kfselect_min_num_frames_between_kfs", 2);
  o.kfselect_backend_max_time_sec = param<double>(pnh, "kfselect_backend_max_time_sec", 3.0);
  o.img_align_max_level = param<int>(pnh, "img_align_max_level", 4);
  o.img_align_min_level = param<int>(pnh, "img_align_min_level", 2);
  o.img_align_robustification = param<bool>(pnh, "img_align_robustification", false);
  o.img_align_use_distortion_jacobian =
      param<bool>(pnh, "img_align_use_distortion_jacobian", false);
  o.img_align_est_illumination_gain =
      param<bool>(pnh, "img_align_est_illumination_gain", false);
  o.img_align_est_illumination_offset =
      param<bool>(pnh, "img_align_est_illumination_offset", false);
  o.poseoptim_thresh = param<double>(pnh, "poseoptim_thresh", 2.0);
  o.update_seeds_with_old_keyframes =
      param<bool>(pnh, "update_seeds_with_old_keyframes", true);
  o.use_async_reprojectors = param<bool>(pnh, "use_async_reprojectors", false);
  o.trace_statistics = param<bool>(pnh, "trace_statistics", false);
  o.backend_scale_stable_thresh =
      param<double>(pnh, "backend_scale_stable_thresh", 0.02);
  o.global_map_lc_timeout_sec_ =
      param<double>(pnh, "global_map_timeout_sec", 2.0);
  o.window_size_ = param<int>(pnh, "window_size", 5);
  o.obs_dev_ = param<double>(pnh, "obs_dev", 2.0);
  return o;
}

DetectorOptions loadDetectorOptions(const YAML::Node& pnh)
{
  DetectorOptions o;
  o.cell_size = param<int>(pnh, "grid_size", 35);
  o.max_level = param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.threshold_primary = param<int>(pnh, "detector_threshold_primary", 10);
  o.threshold_secondary = param<int>(pnh, "detector_threshold_secondary", 200);
  o.threshold_shitomasi = param<int>(pnh, "detector_threshold_shitomasi", 100);
  if(param<bool>(pnh, "use_edgelets", true))
    o.detector_type = DetectorType::kFastGrad;
  else
    o.detector_type = DetectorType::kFast;
  return o;
}

DepthFilterOptions loadDepthFilterOptions(const YAML::Node& pnh)
{
  DepthFilterOptions o;
  o.max_search_level = param<int>(pnh, "n_pyr_levels", 3) - 1;
  o.use_threaded_depthfilter =
      param<bool>(pnh, "use_threaded_depthfilter", true);
  o.seed_convergence_sigma2_thresh =
      param<double>(pnh, "seed_convergence_sigma2_thresh", 200.0);
  o.mappoint_convergence_sigma2_thresh =
      param<double>(pnh, "mappoint_convergence_sigma2_thresh", 500.0);
  o.scan_epi_unit_sphere = param<bool>(pnh, "scan_epi_unit_sphere", false);
  o.affine_est_offset= param<bool>(pnh, "depth_filter_affine_est_offset", true);
  o.affine_est_gain = param<bool>(pnh, "depth_filter_affine_est_gain", false);
  o.max_n_seeds_per_frame = static_cast<size_t>(
        static_cast<double>(param<int>(pnh, "max_fts", 120))
        * param<double>(pnh, "max_seeds_ratio", 3.0));
  o.max_map_seeds_per_frame = static_cast<size_t>(
        static_cast<double>(param<int>(pnh, "max_map_fts", 120)));
  o.extra_map_points =
      param<bool>(pnh, "depth_filter_extra_map_points", false);
  if(param<bool>(pnh, "runlc", false) && !o.extra_map_points)
  {
    LOG(WARNING) << "Loop closure requires extra map points, "
                 << " but the option is not set, overriding to true.";
    o.extra_map_points = true;
  }
  return o;
}

InitializationOptions loadInitializationOptions(const YAML::Node& pnh)
{
  InitializationOptions o;
  o.init_min_features = param<int>(pnh, "init_min_features", 100);
  o.init_min_tracked = param<int>(pnh, "init_min_tracked", 80);
  o.init_min_inliers = param<int>(pnh, "init_min_inliers", 70);
  o.init_min_disparity = param<double>(pnh, "init_min_disparity", 40.0);
  o.init_min_features_factor = param<double>(pnh, "init_min_features_factor", 2.0);
  o.reproj_error_thresh = param<double>(pnh, "reproj_err_thresh", 2.0);
  o.init_disparity_pivot_ratio = param<double>(pnh, "init_disparity_pivot_ratio", 0.5);
  std::string init_method = param<std::string>(pnh, "init_method", "FivePoint");
  if(init_method == "Homography")
    o.init_type = InitializerType::kHomography;
  else if(init_method == "TwoPoint")
    o.init_type = InitializerType::kTwoPoint;
  else if(init_method == "FivePoint")
    o.init_type = InitializerType::kFivePoint;
  else if(init_method == "OneShot")
    o.init_type = InitializerType::kOneShot;
  else
    SVO_ERROR_STREAM("Initialization Method not supported: " << init_method);
  return o;
}

FeatureTrackerOptions loadTrackerOptions(const YAML::Node& pnh)
{
  FeatureTrackerOptions o;
  o.klt_max_level = param<int>(pnh, "klt_max_level", 4);
  o.klt_min_level = param<int>(pnh, "klt_min_level", 0.001);
  return o;
}

ReprojectorOptions loadReprojectorOptions(const YAML::Node& pnh)
{
  ReprojectorOptions o;
  o.max_n_kfs = param<int>(pnh, "reprojector_max_n_kfs", 5);
  o.max_n_features_per_frame = param<int>(pnh, "max_fts", 160);
  o.cell_size = param<int>(pnh, "grid_size", 35);
  o.reproject_unconverged_seeds =
      param<bool>(pnh, "reproject_unconverged_seeds", true);
  o.max_unconverged_seeds_ratio =
      param<double>(pnh, "max_unconverged_seeds_ratio", -1.0);
  o.min_required_features =
      param<int>(pnh, "quality_min_fts", 50);
  o.seed_sigma2_thresh =
      param<double>(pnh, "seed_convergence_sigma2_thresh", 200.0);

  o.affine_est_offset =
      param<bool>(pnh, "reprojector_affine_est_offset", true);
  o.affine_est_gain =
      param<bool>(pnh, "reprojector_affine_est_gain", false);
  o.max_fixed_landmarks =
      param<int>(pnh, "reprojector_max_fixed_landmarks", 50);
  o.max_n_global_kfs =
      param<int>(pnh, "reprojector_max_n_global_kfs", 20);
  o.use_kfs_from_global_map =
      param<bool>(pnh, "reprojector_use_kfs_from_global_map", false);
  o.fixed_lm_grid_size =
      param<int>(pnh, "reprojector_fixed_lm_grid_size", 50);

  return o;
}

CameraBundle::Ptr loadCameraFromYaml(const YAML::Node& pnh)
{
  std::string calib_file = param<std::string>(pnh, "calib_file", "~/cam.yaml");
  CameraBundle::Ptr ncam = CameraBundle::loadFromYaml(calib_file);
  std::cout << "loaded " << ncam->numCameras() << " cameras";
  for(const auto& cam : ncam->getCameraVector())
    cam->printParameters(std::cout, "");
  return ncam;
}

StereoTriangulationOptions loadStereoOptions(const YAML::Node& pnh)
{
  StereoTriangulationOptions o;
  o.triangulate_n_features = param<int>(pnh, "max_fts", 120);
  o.max_depth_inv = param<double>(pnh, "max_depth_inv", 1.0/50.0);
  o.min_depth_inv = param<double>(pnh, "min_depth_inv", 1.0/0.5);
  o.mean_depth_inv = param<double>(pnh, "mean_depth_inv", 1.0/2.0);
  return o;
}

ImuHandler::Ptr getImuHandler(const YAML::Node& pnh)
{
  std::string calib_file = param<std::string>(pnh, "calib_file", "");
  ImuCalibration imu_calib = ImuHandler::loadCalibrationFromFile(calib_file);
  imu_calib.print("Loaded IMU Calibration");
  ImuInitialization imu_init = ImuHandler::loadInitializationFromFile(calib_file);
  imu_init.print("Loaded IMU Initialization");
  IMUHandlerOptions options;
  options.temporal_stationary_check =
      param<bool>(pnh, "imu_temporal_stationary_check", false);
  options.temporal_window_length_sec_ =
      param<double>(pnh, "imu_temporal_window_length_sec", 0.5);
  options.stationary_acc_sigma_thresh_ =
      param<double>(pnh, "stationary_acc_sigma_thresh", 0.0);
  options.stationary_gyr_sigma_thresh_ =
      param<double>(pnh, "stationary_gyr_sigma_thresh", 0.0);
  ImuHandler::Ptr imu_handler(new ImuHandler(imu_calib, imu_init, options));
  return imu_handler;
}

void setInitialPose(const YAML::Node& pnh, FrameHandlerBase& vo)
{
  Transformation T_world_imuinit(
        Quaternion(param<double>(pnh, "T_world_imuinit/qw", 1.0),
                   param<double>(pnh, "T_world_imuinit/qx", 0.0),
                   param<double>(pnh, "T_world_imuinit/qy", 0.0),
                   param<double>(pnh, "T_world_imuinit/qz", 0.0)),
        Vector3d(param<double>(pnh, "T_world_imuinit/tx", 0.0),
                 param<double>(pnh, "T_world_imuinit/ty", 0.0),
                 param<double>(pnh, "T_world_imuinit/tz", 0.0)));
  vo.setInitialImuPose(T_world_imuinit);
}


FrameHandlerMono::Ptr makeMono(const YAML::Node& pnh, const CameraBundlePtr& cam = nullptr)
{
  // Create camera
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);
  if (ncam->numCameras() > 1)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(1);
  }

  // Init VO
  FrameHandlerMono::Ptr vo =
      std::make_shared<FrameHandlerMono>(
        loadBaseOptions(pnh, false),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        loadInitializationOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}

FrameHandlerStereo::Ptr makeStereo(const YAML::Node& pnh, const CameraBundlePtr& cam = nullptr)
{
  // Load cameras
  CameraBundle::Ptr ncam = (cam) ? cam : loadCameraFromYaml(pnh);
  if (ncam->numCameras() > 2)
  {
    LOG(WARNING) << "Load more cameras than needed, will erase from the end.";
    ncam->keepFirstNCams(2);
  }

  // Init VO
  InitializationOptions init_options = loadInitializationOptions(pnh);
  init_options.init_type = InitializerType::kStereo;
  FrameHandlerStereo::Ptr vo =
      std::make_shared<FrameHandlerStereo>(
        loadBaseOptions(pnh, true),
        loadDepthFilterOptions(pnh),
        loadDetectorOptions(pnh),
        init_options,
        loadStereoOptions(pnh),
        loadReprojectorOptions(pnh),
        loadTrackerOptions(pnh),
        ncam);

  // Get initial position and orientation of IMU
  setInitialPose(pnh, *vo);

  return vo;
}


}}
