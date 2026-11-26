// Four captured sensor-only inputs exercise the production raw-factor adapter.
// This packet lacks a bootstrap posterior and is too short for metric readiness.
// Historical poses and metric velocity/bias means are deliberately not priors.
#include "joint_runtime.h"
#include <svo/common/camera.h>
#include <svo/common/point.h>
#include <svo/imu_handler.h>
#include <svo/schur_vins.h>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <tuple>

namespace {
void require(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
Eigen::Vector3d vec(const YAML::Node& n) {
  return {n[0].as<double>(),n[1].as<double>(),n[2].as<double>()};
}
svo::Transformation pose(const YAML::Node& n) {
  const auto q=n["q"];
  return svo::Transformation(Eigen::Quaterniond(q[0].as<double>(),q[1].as<double>(),
                                               q[2].as<double>(),q[3].as<double>()),vec(n["p"]));
}
}
int main(int argc,char** argv) {
  try {
    require(argc==3,"usage: schur_joint_packet euroc_mono.yaml sensor-only-first-four.json");
    const auto cameras=svo::CameraBundle::loadFromYaml(argv[1]);
    require(cameras->getNumCameras()==1,"packet test requires the mono calibration");
    const auto calibration=svo::ImuHandler::loadCalibrationFromFile(argv[1]);
    const auto initial=svo::ImuHandler::loadInitializationFromFile(argv[1]);
    const auto input=YAML::LoadFile(argv[2]);
    require(input["steps"].size()==4,"expected exactly four captured current inputs");
    require(!input["unit_sphere"].as<bool>() && input["huber_a"].as<double>()==1.5 &&
        input["huber_b"].as<double>()==2.25,"packet visual model differs from production contract");
    schur_vins::SchurVINS filter;
    filter.InitExtrinsic(cameras);filter.InitMaxState(5);
    filter.InitFocalLength(input["focal"].as<double>());
    filter.InitObsStddev(input["obs_std"].as<double>());
    rm27::MonoAlignment alignment;
    std::map<int,svo::PointPtr> points;
    std::set<std::tuple<int64_t,int,size_t>> keys;
    size_t expected_pixels=0;
    svo::FrameBundlePtr last;
    for(const auto& step:input["steps"]) {
      for(const auto& node:step["new_points"]) {
        const int id=node["id"].as<int>();
        require(points.emplace(id,std::make_shared<svo::Point>(id,vec(node["p"]))).second,
                "captured landmark seed repeated");
      }
      const int id=step["frame_id"].as<int>();
      const auto raw_frame=input["frames"][std::to_string(id)];
      svo::FramePtr frame(new svo::Frame());
      frame->id_=id;frame->bundle_id_=step["bundle"].as<int>();
      frame->timestamp_=step["sensor_ns"].as<int64_t>();
      frame->cam_=cameras->getCameraShared(0);
      frame->setNFrameIndex(raw_frame["camera"].as<int>());
      frame->T_f_w_=pose(raw_frame["T_W_C"]).inverse();
      frame->set_T_cam_imu(pose(raw_frame["T_C_B"]));
      size_t size=0;
      for(const auto& ft:step["features"])size=std::max(size,ft["keypoint"].as<size_t>()+1);
      frame->resizeFeatureStorage(size);frame->num_features_=size;
      for(const auto& ft:step["features"]) {
        const size_t k=ft["keypoint"].as<size_t>();
        frame->type_vec_[k]=svo::FeatureType::kCorner;
        frame->level_vec_[k]=ft["level"].as<int>();
        frame->f_vec_.col(k)=vec(ft["raw_bearing"]);
        frame->landmark_vec_[k]=points.at(ft["point_id"].as<int>());
        require(keys.emplace(frame->timestamp_,frame->getNFrameIndex(),k).second,
                "captured current pixel identity repeated");
        ++expected_pixels;
      }
      last=std::make_shared<svo::FrameBundle>(std::vector<svo::FramePtr>{frame},
                                            step["bundle"].as<int>());
      for(const auto& m:step["imu"]) {
        last->imu_datas_.emplace_back(m[0].as<double>(),
            Eigen::Vector3d(m[1].as<double>(),m[2].as<double>(),m[3].as<double>()),
            Eigen::Vector3d(m[4].as<double>(),m[5].as<double>(),m[6].as<double>()));
      }
      last->imu_ready_=true;alignment.add(last);
      require(!filter.BootstrapJoint(last,calibration,initial,alignment),
              "four mean-state inputs bypassed physical bootstrap qualification");
      require(!filter.MetricReady() && !filter.LastVisualCorrectionAccepted(),
              "short packet claimed metric readiness or a fresh visual correction");
    }
    require(filter.JointModel()!=nullptr,"production bootstrap did not instantiate raw joint model");
    const auto& window=filter.JointModel()->window();
    const auto status=window.status();
    require(window.frames().size()==4 && status.active_imu_edges==3 &&
        status.active_pixels==expected_pixels && status.active_pixels==keys.size() &&
        status.active_points==points.size() && status.absorbed_pixels==0 &&
        status.absorbed_imu_edges==0 && status.dropped_pixels==0,
        "packet raw factors were lost, duplicated or replaced by historical fixed rows");
    require((window.frames().front().v_W-initial.velocity).norm()==0 &&
        (window.frames().front().ba-initial.acc_bias).norm()==0 &&
        (window.frames().front().bg-initial.omega_bias).norm()==0,
        "captured metric velocity/bias means leaked into configured initialization");
    require(!filter.BootstrapJoint(last,calibration,initial,alignment) &&
        filter.JointModel()->window().status().active_pixels==expected_pixels &&
        filter.JointModel()->window().status().active_imu_edges==3,
        "replaying an existing raw bundle added independent likelihood");
    for(const auto& point:points)require(!point.second->ekf_init,
        "raw packet adapter installed an independent fake point covariance");
    // Copy the admitted production graph, preserving its configured initial
    // prior. Optimize exactly those raw factors without altering readiness.
    auto objective=window;
    const auto before=objective.linearize();
    require(before.finite,"captured raw joint objective is nonfinite");
    Eigen::MatrixXd H;Eigen::VectorXd b;
    require(schur_vins::JointWindow::schur(before,&H,&b) && H.allFinite() && b.allFinite(),
            "captured camera Schur objective cannot eliminate its own point blocks");
    const auto solved=objective.optimize();
    require(solved.finite && std::isfinite(solved.final_cost) &&
        solved.final_cost<=before.cost+1e-10*(1+before.cost),
        "production optimizer did not preserve monotonic common raw-factor cost");
    require(!filter.MetricReady() && !filter.LastVisualCorrectionAccepted(),
            "diagnostic graph optimization changed production readiness");
    std::cout<<std::setprecision(17)
        <<"{\"PASS\":true,\"scope\":\"FOUR_FRAME_RAW_CONSUMPTION_AND_REQUIRED_REFUSAL\","
        <<"\"frames\":4,\"imu_edges\":3,\"unique_pixels\":"<<expected_pixels
        <<",\"points\":"<<points.size()<<",\"initial_cost\":"<<before.cost
        <<",\"final_cost\":"<<solved.final_cost<<",\"metric_ready\":false}\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"FAIL joint packet: "<<error.what()<<"\n";return 1;
  }
}
