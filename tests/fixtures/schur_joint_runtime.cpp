// Production raw-factor adapter with independent analytic camera/IMU motion.
// Synthetic behavior coverage only; no dataset/hardware accuracy qualification.
#include "joint_runtime.h"
#include <svo/common/camera.h>
#include <svo/common/point.h>
#include <svo/imu_handler.h>
#include <svo/frame_handler_mono.h>
#include <svo/schur_vins.h>
#include <svo/map.h>
#include <svo/initialization.h>
#include <svo/direct/depth_filter.h>
#include <svo/direct/feature_detection.h>
#include <svo/reprojector.h>
#include <svo/tracker/feature_tracking_types.h>
#include <algorithm>
#include <vikit/cameras/camera_geometry_base.h>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
using V3 = Eigen::Vector3d;
const V3 gravity(0,0,-9.81007), ba(.02,-.01,.01), bg(.001,-.002,.001);
constexpr double scale = 2.3, first_time = 1., image_dt = .05, imu_dt = .005;
void require(bool value,const char* message) {
  if(!value)throw std::runtime_error(message);
}
Eigen::Quaterniond orientation(double t,bool moving) {
  return moving ? rm27::expq(V3(.25*std::sin(1.9*t),.22*std::sin(1.3*t),
                                 .3*std::sin(.9*t))) : Eigen::Quaterniond::Identity();
}
V3 position(double t,bool moving) {
  return moving ? V3(.2*t+.35*(1-std::cos(2.1*t)), -.1*t+.25*std::sin(1.7*t),
                      .05*t+.2*(1-std::cos(1.3*t))) : V3::Zero();
}
V3 velocity(double t,bool moving) {
  return moving ? V3(.2+.735*std::sin(2.1*t),-.1+.425*std::cos(1.7*t),
                      .05+.26*std::sin(1.3*t)) : V3::Zero();
}
V3 acceleration(double t,bool moving) {
  return moving ? V3(1.5435*std::cos(2.1*t),-.7225*std::sin(1.7*t),
                      .338*std::cos(1.3*t)) : V3::Zero();
}
struct Scenario {
  svo::CameraBundlePtr cameras;
  std::vector<svo::PointPtr> points;
  std::vector<V3> truth;
  V3 origin;
  bool moving;
  Scenario(const svo::CameraBundlePtr& c,bool m):cameras(c),moving(m) {
    const auto T_B_C=cameras->get_T_C_B(0).inverse();
    origin=T_B_C.getPosition();
    for(int y=0;y<4;++y)for(int x=0;x<6;++x) {
      const V3 world=T_B_C*V3((x-2.5)*.22,(y-1.5)*.2,4+.05*((x+y)%3));
      truth.push_back(world);
      points.emplace_back(new svo::Point(origin+(world-origin)/scale));
    }
  }
  svo::FrameBundlePtr frame(int k) {
    const double t=k*image_dt;
    const auto q=orientation(t,moving);
    const auto T_B_C=cameras->get_T_C_B(0).inverse();
    const V3 camera_true=position(t,moving)+q*T_B_C.getPosition();
    const V3 camera_visual=origin+(camera_true-origin)/scale;
    const svo::Transformation T_W_C(q*T_B_C.getEigenQuaternion(),camera_visual);
    svo::FramePtr f(new svo::Frame(k,int64_t(std::llround((first_time+t)*1e9)),
                                  cameras->getCameraShared(0),T_W_C));
    f->set_T_cam_imu(cameras->get_T_C_B(0));f->setNFrameIndex(0);
    f->resizeFeatureStorage(points.size());f->num_features_=points.size();
    const svo::Transformation T_C_W_true=(svo::Transformation(q,position(t,moving))*T_B_C).inverse();
    for(size_t i=0;i<points.size();++i) {
      if(!points[i])continue;  // Raw latent factors may outlive the SVO point object.
      const V3 pc=T_C_W_true*truth[i];
      require(pc.z()>0,"synthetic point behind camera");
      const auto intrinsic=f->cam()->getIntrinsicParameters();
      f->px_vec_.col(i)=Eigen::Vector2d(intrinsic[0]*pc.x()/pc.z()+intrinsic[2],
                                       intrinsic[1]*pc.y()/pc.z()+intrinsic[3]);
      f->f_vec_.col(i)=pc.normalized();f->landmark_vec_[i]=points[i];
      f->track_id_vec_[i]=points[i]->id();f->type_vec_[i]=svo::FeatureType::kCorner;
      f->level_vec_[i]=0;points[i]->addObservation(f,i);
    }
    svo::FrameBundlePtr b(new svo::FrameBundle({f}));
    for(int j=-10;j<=1;++j) {
      const double tm=t+j*imu_dt;
      const auto qm=orientation(tm,moving);
      const V3 omega=rm27::logq(qm.conjugate()*orientation(tm+1e-6,moving))/1e-6;
      b->imu_datas_.emplace_back(first_time+tm,omega+bg,
                                  qm.conjugate()*(acceleration(tm,moving)-gravity)+ba);
    }
    b->imu_ready_=true;
    return b;
  }
};
class RetirementProbe : public svo::FrameHandlerMono {
 public:
  explicit RetirementProbe(const svo::CameraBundlePtr& cameras)
    : FrameHandlerMono(baseOptions(),depthOptions(),svo::DetectorOptions(),
        svo::InitializationOptions(),svo::ReprojectorOptions(),
        svo::FeatureTrackerOptions(),cameras) {}
  void addOldKeyframe(const svo::FramePtr& frame) {
    frame->setKeyframe();map_->addKeyframe(frame,false);
  }
  bool admit(const svo::FrameBundlePtr& frame,const svo::ImuCalibration& calibration,
             const svo::ImuInitialization& initial,const rm27::MonoAlignment& alignment) {
    new_frames_=frame;
    return schur_vins_->BootstrapJoint(frame,calibration,initial,alignment);
  }
  void retireUnsupportedGeometry() { transformMetricMap(rm27::MonoAlignmentResult()); }
  void releaseLastRemovedOwner() {
    svo::FramePtr removed;map_->getLastRemovedKF(&removed);removed.reset();
  }
  const svo::FramePtr& current() const { return new_frames_->frames_[0]; }
 private:
  static svo::BaseOptions baseOptions() {
    svo::BaseOptions options;options.window_size_=5;options.obs_dev_=2.;return options;
  }
  static svo::DepthFilterOptions depthOptions() {
    svo::DepthFilterOptions options;
    options.use_threaded_depthfilter=false;
    // Existing points exceed this count, so the real addKeyframe path has no
    // new image seeds to detect. This test isolates geometry/reference lifetime.
    options.max_n_seeds_per_frame=1;options.extra_map_points=false;return options;
  }
};
void runGeometryRetirement(const svo::CameraBundlePtr& cameras,const std::string& yaml) {
  Scenario scenario(cameras,true);
  auto old=scenario.frame(-1);
  const int retired_id=old->at(0)->id();
  const std::weak_ptr<svo::Frame> retired_owner=old->at(0);
  RetirementProbe frontend(cameras);frontend.addOldKeyframe(old->at(0));
  rm27::MonoAlignment alignment;
  const auto calibration=svo::ImuHandler::loadCalibrationFromFile(yaml);
  const auto initial=svo::ImuHandler::loadInitializationFromFile(yaml);
  bool accepted=false;
  for(int k=0;k<=32;++k) {
    auto frame=scenario.frame(k);alignment.add(frame);
    if(frontend.admit(frame,calibration,initial,alignment)) {accepted=true;break;}
  }
  require(accepted,"frontend retirement setup did not reach physical raw bootstrap");
  frontend.retireUnsupportedGeometry();
  // Production retains the removed keyframe until the next kKeyframe event.
  // Releasing that final owner exposed the first real-sequence matching loss.
  old.reset();frontend.releaseLastRemovedOwner();
  require(retired_owner.expired(),"test accidentally kept retired keyframe owner alive");
  for(const auto& point:scenario.points) {
    require(std::none_of(point->obs_.begin(),point->obs_.end(),
        [&](const svo::KeypointIdentifier& observation){return observation.frame_id==retired_id;}),
        "frontend geometry retirement left a stale observation on a surviving point");
    svo::FramePtr reference;size_t feature=0;
    require(point->getCloseViewObs(frontend.current()->pos(),reference,feature) &&
        reference && reference->id()!=retired_id && feature<reference->numFeatures() &&
        reference->landmark_vec_[feature]==point,
        "retired keyframe release prevented matching a surviving valid point");
  }
  std::cout<<"PASS actual frontend geometry retirement and delayed owner release\n";
}

void runExpiredSeed(const svo::CameraBundlePtr& cameras,const std::string& yaml) {
  Scenario scenario(cameras,true);
  const int latent_id=scenario.points.front()->id();
  const std::weak_ptr<svo::Point> weak=scenario.points.front();
  const auto calibration=svo::ImuHandler::loadCalibrationFromFile(yaml);
  const auto initial=svo::ImuHandler::loadInitializationFromFile(yaml);
  schur_vins::JointRuntime runtime(calibration,initial,400,2,5);
  rm27::MonoAlignment alignment;
  std::vector<svo::FrameBundlePtr> cached;
  bool accepted=false;
  for(int k=0;k<=32;++k) {
    if(k==15) {
      for(const auto& frame:cached)frame->at(0)->landmark_vec_[0].reset();
      scenario.points[0].reset();
      require(weak.expired(),"test kept expired SVO point alive");
      require(runtime.window().landmarks().count(latent_id)==1,
              "removing frontend owner also removed independent raw latent variable");
    }
    auto frame=scenario.frame(k);cached.push_back(frame);alignment.add(frame);
    if(runtime.bootstrap(frame,alignment)){accepted=true;break;}
  }
  require(accepted && runtime.ready(),"expired-point bootstrap failed original physical gates");
  const auto& seeded=runtime.seedReport();
  require(seeded.expired_points>=1,"seed audit did not see expired frontend point");
  require(seeded.max_similarity_error<1e-10,
          "metric seeding left a latent point in visual-scale coordinates");
  require(runtime.window().landmarks().count(latent_id)==1 &&
      runtime.window().status().active_pixels==
          15*scenario.points.size()+(cached.size()-15)*(scenario.points.size()-1),
      "expired frontend object lost or duplicated its earlier raw pixel factors");
  std::cout<<"PASS expired frontend point retains correctly scaled raw latent factors\n";
}

void run(const svo::CameraBundlePtr& cameras,bool moving,const std::string& yaml,bool production_priors) {
  Scenario scenario(cameras,moving);
  svo::ImuCalibration calibration;
  calibration.acc_noise_density=.01;calibration.gyro_noise_density=.001;
  calibration.acc_bias_random_walk_sigma=.001;calibration.gyro_bias_random_walk_sigma=.0001;
  svo::ImuInitialization initial;
  initial.velocity=velocity(0,moving);initial.acc_bias=ba;initial.omega_bias=bg;
  if(production_priors) {
    calibration=svo::ImuHandler::loadCalibrationFromFile(yaml);
    initial=svo::ImuHandler::loadInitializationFromFile(yaml);
  }
  schur_vins::JointRuntime runtime(calibration,initial,400,2,5);
  rm27::MonoAlignment alignment;
  std::vector<svo::FrameBundlePtr> input;
  for(int k=0;k<=32;++k) {
    auto b=scenario.frame(k);input.push_back(b);alignment.add(b);
    const bool accepted=runtime.bootstrap(b,alignment);
    if(k<30)require(!accepted,"metric readiness bypassed original span/window gate");
    if(accepted)break;
  }
  const auto before=runtime.window().status();
  require(before.active_imu_edges+1==runtime.window().frames().size(),
          "first-frame interval rejected or raw IMU edge missing");
  require(before.active_pixels==runtime.window().frames().size()*scenario.points.size(),
          "bootstrap did not preserve every unique raw pixel exactly once");
  require(before.absorbed_pixels==0,"short bootstrap unexpectedly consumed raw pixels");
  if(!moving) {
    require(!runtime.ready(),"stationary unobservable bootstrap claimed metric readiness");
    std::cout<<"PASS stationary raw joint bootstrap remains rejected\n";
    return;
  }
  require(runtime.ready(),"exciting analytic motion failed physical joint bootstrap");
  runtime.commitGeometry();
  const auto& last=runtime.window().frames().back();
  const double t=last.timestamp-first_time;
  require((last.p_WB-position(t,true)).norm()<.06,"joint bootstrap did not recover metric translation");
  require((runtime.window().gravity()-gravity).norm()<.2,"joint bootstrap gravity incorrect");
  double point_error=0;
  for(size_t i=0;i<scenario.points.size();++i) {
    require(!scenario.points[i]->ekf_init,"raw joint adapter installed fake independent point prior");
    point_error=std::max(point_error,(scenario.points[i]->pos()-scenario.truth[i]).norm());
  }
  require(point_error<.15,"joint point back-substitution did not recover metric geometry");
  require(runtime.window().linearize().finite,"accepted bootstrap has no finite persisted objective");
  const int valid=runtime.backward(input.back());
  const auto repeated=runtime.window().status();
  require(valid==0 && !runtime.corrected() && repeated.active_pixels==before.active_pixels &&
      repeated.active_imu_edges==before.active_imu_edges,
      "repeated update duplicated likelihood or claimed fresh correction");
  runtime.manage();
  const auto marginalized=runtime.window().status();
  require(runtime.window().frames().size()==5,"fixed-lag marginalization retained wrong frames");
  require(marginalized.active_pixels==0 && marginalized.absorbed_pixels==before.active_pixels,
          "landmark closure did not absorb retained-frame rows exactly once");
  const int next_k=int(std::llround(t/image_dt))+1;
  auto next=scenario.frame(next_k);
  runtime.forward(next);
  require(runtime.backward(next)>0,"future free segments failed after marginalization");
  require(!runtime.corrected(),"one-view free segments invented visual state correction");
  const auto restarted=runtime.window().status();
  require(restarted.active_pixels==scenario.points.size() &&
      restarted.absorbed_pixels==before.active_pixels,
      "new point segment reused an absorbed old pixel");
  std::cout<<(production_priors?"PASS YAML cold physical bootstrap":"PASS analytic physical bootstrap")
           <<", repeated factors and segment retirement"
           <<" point_error_m="<<point_error<<"\n";
}
}
int main(int argc,char** argv) {
  try {
    if(argc!=2)throw std::runtime_error("expected mono calibration YAML");
    const auto cameras=svo::CameraBundle::loadFromYaml(argv[1]);
    runGeometryRetirement(cameras,argv[1]);
    runExpiredSeed(cameras,argv[1]);
    run(cameras,true,argv[1],false);run(cameras,true,argv[1],true);
    run(cameras,false,argv[1],true);return 0;
  } catch(const std::exception& e) {
    std::cerr<<"FAIL joint runtime: "<<e.what()<<"\n";return 1;
  }
}
