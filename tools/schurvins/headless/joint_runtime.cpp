#include "joint_runtime.h"
#include <svo/common/point.h>
#include <vikit/cameras/camera_geometry_base.h>
#include <svo/rm27_diagnostics.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <set>
#include <fstream>
#include <iomanip>
#include <cstdlib>
namespace schur_vins {
namespace ji = svo::headless::joint_imu;
namespace {
JointWindow::Options options(const svo::ImuCalibration& c) {
  JointWindow::Options o;
  o.imu.sigma_acc=c.acc_noise_density; o.imu.sigma_gyro=c.gyro_noise_density;
  o.imu.sigma_ba=c.acc_bias_random_walk_sigma;
  o.imu.sigma_bg=c.gyro_bias_random_walk_sigma;
  o.imu.max_gap=rm27::MonoAlignmentPolicy().max_imu_gap_s;
  return o;
}
void captureFailure(const JointWindow& window,const svo::FrameBundle::Ptr& b,bool bootstrap,bool input) {
  static bool snapshots[4]={false,false,false,false};
  bool& saved=snapshots[(bootstrap?2:0)+(input?1:0)];if(saved)return;
  const char* path=std::getenv("RM27_NATIVE_EVENTS");if(!path)return;
  const std::string full(path);const auto slash=full.find_last_of('/');
  const std::string name=full.substr(0,slash+1)+std::string(bootstrap?"joint-bootstrap-":"joint-tracking-")+(input?"input.yaml":"iterate.yaml");
  std::string error;saved=window.writeSnapshot(name,&error);
  rm27::event(saved?"joint_snapshot_saved":"joint_snapshot_failed",b,{double(bootstrap)});
}
JointWindow::Frame state(const svo::FrameBundle::Ptr& b) {
  JointWindow::Frame f; f.ns=b->getMinTimestampNanoseconds();f.timestamp=b->getMinTimestampSeconds();
  f.q_WB=b->get_T_W_B().getEigenQuaternion();f.p_WB=b->get_T_W_B().getPosition();return f;
}
}
JointRuntime::JointRuntime(const svo::ImuCalibration& c,
    const svo::ImuInitialization& i,double focal,double sigma,int n)
  :window_(options(c)),calibration_(options(c).imu),initial_(i),focal_(focal),sigma_(sigma),tracking_frames_(n) {}

bool JointRuntime::append(const svo::FrameBundle::Ptr& b,bool metric) {
  const int64_t ns=b->getMinTimestampNanoseconds();
  if(frames_.count(ns))return true;
  JointWindow next=window_;
  auto f=state(b);
  if(seeded_ && !metric) {
    const auto tbc=b->at(0)->T_imu_cam().getPosition();
    f.p_WB=origin_+bootstrap_scale_*(f.p_WB+f.q_WB*tbc-origin_)-f.q_WB*tbc;
  }
  if(!next.frames().empty()) {
    const auto& prev=next.frames().back();f.ba=prev.ba;f.bg=prev.bg;f.v_W=prev.v_W;
  } else {f.v_W=initial_.velocity;f.ba=initial_.acc_bias;f.bg=initial_.omega_bias;}
  std::string error;
  if(!next.addFrame(f,next.frames().empty()?ji::Measurements():ji::fromSvoMeasurements(b->imu_datas_),&error)) {
    rm27::event(("joint_append_"+error).c_str(),b);return false;
  }
  if(frames_.empty()) {
    origin_=b->at(0)->T_world_cam().getPosition();
    if(!next.setInitialPrior(initial_.velocity,initial_.acc_bias,initial_.omega_bias,
        initial_.velocity_sigma,initial_.acc_bias_sigma,initial_.omega_bias_sigma,&error))return false;
  }
  for(const auto& camera:b->frames_) {
    auto intrinsic=camera->cam_->getIntrinsicParameters();
    for(size_t k=0;k<camera->numFeatures();++k) {
      const auto& p=camera->landmark_vec_[k];if(!p || camera->type_vec_[k]==svo::FeatureType::kOutlier)continue;
      next.addLandmark(p->id(),seeded_ && !metric ? (origin_+bootstrap_scale_*(p->pos()-origin_)).eval() : p->pos(),&error);points_[p->id()]=p;
      JointWindow::Observation o;o.frame_ns=ns;o.point_id=p->id();
      o.camera_id=camera->getNFrameIndex();o.feature_id=k;
      o.bearing=camera->f_vec_.col(k);o.q_CB=camera->T_cam_imu().getEigenQuaternion();
      o.t_CB=camera->T_cam_imu().getPosition();
      o.focal=intrinsic.head<2>();o.sigma_pixels=sigma_;o.level=camera->level_vec_[k];
      if(!next.addObservation(o,&error)) {
        rm27::event(("joint_pixel_"+error).c_str(),b);return false;
      }
    }
  }
  window_=std::move(next);frames_[ns]=b;
  return true;
}

bool JointRuntime::seedMetric(const rm27::MonoAlignmentResult& a) {
  const double scale=a.scale>0 && a.scale<=100?a.scale:1.;
  const Eigen::Vector3d g=a.gravity.norm()>0 && a.gravity.allFinite()?a.gravity:Eigen::Vector3d(0,0,-9.81007);
  JointWindow trial=window_;std::string error;
  if(!trial.setGravity(g,&error))return false;
  auto seeds=window_.frames();
  std::map<int64_t,JointWindow::Frame> canonical_frames;
  for(size_t k=0;k<seeds.size();++k) {
    auto& f=seeds[k];const auto& b=frames_.at(f.ns);f=state(b);canonical_frames[f.ns]=f;
    const auto tbc=b->at(0)->T_imu_cam().getPosition();
    const Eigen::Vector3d camera=f.p_WB+f.q_WB*tbc;
    f.p_WB=origin_+scale*(camera-origin_)-f.q_WB*tbc;
    f.ba=a.ba.allFinite() && a.ba.norm()<=1.?a.ba:initial_.acc_bias;
    f.bg=a.bg.allFinite() && a.bg.norm()<=.2?a.bg:initial_.omega_bias;
  }
  for(size_t k=0;k<seeds.size();++k) {
    auto& f=seeds[k];
    if(k+1<seeds.size())f.v_W=(seeds[k+1].p_WB-f.p_WB)/((seeds[k+1].ns-f.ns)*1e-9);
    else if(k)f.v_W=seeds[k-1].v_W;
    if(!trial.setFrameSeed(f,&error))return false;
  }
  std::map<int64_t,Eigen::Vector3d> canonical_points,legacy_points;
  seed_report_=SeedReport();
  for(const auto& x:window_.landmarks()) {
    Eigen::Vector3d canonical=x.second;bool expired=true;
    const auto it=points_.find(x.first);
    if(it!=points_.end())if(auto p=it->second.lock()){canonical=p->pos();expired=false;}
    if(expired)++seed_report_.expired_points;
    canonical_points[x.first]=canonical;
    // The old adapter skipped expired SVO owners even though their raw-factor
    // landmark lives independently. Its counterfactual is diagnostic only.
    legacy_points[x.first]=expired?x.second:(origin_+scale*(canonical-origin_)).eval();
    if(!trial.setLandmarkSeed(x.first,origin_+scale*(canonical-origin_),&error))return false;
  }
  std::map<int64_t,JointWindow::Frame> actual;
  for(const auto& f:trial.frames())actual[f.ns]=f;
  bool written=false;
  static bool captured=false;
  for(const auto& o:trial.observations()) {
    const auto& f=actual.at(o.frame_ns);const auto& source=canonical_frames.at(o.frame_ns);
    const Eigen::Vector3d source_pc=o.q_CB*(source.q_WB.conjugate()*(canonical_points.at(o.point_id)-source.p_WB))+o.t_CB;
    const Eigen::Vector3d pc=o.q_CB*(f.q_WB.conjugate()*(trial.landmarks().at(o.point_id)-f.p_WB))+o.t_CB;
    const Eigen::Vector3d legacy_pc=o.q_CB*(f.q_WB.conjugate()*(legacy_points.at(o.point_id)-f.p_WB))+o.t_CB;
    const double discrepancy=(pc-scale*source_pc).norm();
    seed_report_.max_similarity_error=std::max(seed_report_.max_similarity_error,discrepancy);
    if(!captured && (legacy_pc.z()<=0 || pc.z()<=0 || (legacy_pc-pc).norm()>1e-8)) {
      const char* path=std::getenv("RM27_NATIVE_EVENTS");
      if(path) {
        const std::string full(path);const auto slash=full.find_last_of('/');
        std::ofstream out(full.substr(0,slash+1)+"joint-seed-counterexample.json");
        const auto vector=[&](const Eigen::Vector3d& v){out<<'['<<v.x()<<','<<v.y()<<','<<v.z()<<']';};
        out<<std::setprecision(17)<<"{\"scope\":\"FIRST_CANONICAL_SEED_SIMILARITY_WITNESS\",\"point_id\":"<<o.point_id
           <<",\"frame_ns\":"<<o.frame_ns<<",\"scale_seed\":"<<scale<<",\"expired_points\":"<<seed_report_.expired_points
           <<",\"canonical_pc\":";vector(source_pc);out<<",\"actual_pc\":";vector(pc);
        out<<",\"legacy_skipped_point_pc\":";vector(legacy_pc);out<<",\"origin\":";vector(origin_);
        out<<",\"canonical_point\":";vector(canonical_points.at(o.point_id));out<<",\"scaled_point\":";vector(trial.landmarks().at(o.point_id));
        out<<",\"similarity_error\":"<<discrepancy<<"}\n";written=bool(out);captured=written;
      }
    }
  }
  window_=std::move(trial);bootstrap_scale_=scale;seeded_=true;
  rm27::event("joint_seed_geometry",frames_.rbegin()->second,{double(seed_report_.expired_points),seed_report_.max_similarity_error,double(written)});
  return true;
}

bool JointRuntime::qualify(const rm27::MonoAlignment& raw,std::string* reason) {
  const auto& policy=raw.policy;const auto& fs=window_.frames();
  if(fs.size()<policy.min_frames || (fs.back().ns-fs.front().ns)*1e-9<policy.min_span_s) {
    *reason="WINDOW_SHORT";return false;
  }
  const auto g=window_.gravity();
  if(std::abs(g.norm()-policy.gravity_mps2)>policy.gravity_tolerance_mps2) {*reason="GRAVITY_BOUND";return false;}
  double residual=0,rotation=0,travel=0;size_t count=0;
  std::vector<Eigen::Vector3d> accelerations;
  for(size_t k=0;k<fs.size();++k) {
    if(fs[k].ba.norm()>policy.max_accel_bias_mps2 || fs[k].bg.norm()>policy.max_gyro_bias_rps) {
      *reason="BIAS_BOUND";return false;
    }
    travel=std::max(travel,(fs[k].p_WB-fs.front().p_WB).norm());
    if(!k)continue;
    ji::Preintegration pi;std::string error;
    const auto b=frames_.at(fs[k].ns);
    if(!ji::preintegrate(ji::fromSvoMeasurements(b->imu_datas_),fs[k-1].timestamp,
        fs[k].timestamp,fs[k-1].ba,fs[k-1].bg,calibration_,&pi,&error)) {
      *reason="IMU_MODEL";return false;
    }
    ji::State i,j;
    i.q_WB=fs[k-1].q_WB;i.p_WB=fs[k-1].p_WB;i.v_W=fs[k-1].v_W;i.ba=fs[k-1].ba;i.bg=fs[k-1].bg;
    j.q_WB=fs[k].q_WB;j.p_WB=fs[k].p_WB;j.v_W=fs[k].v_W;j.ba=fs[k].ba;j.bg=fs[k].bg;
    const auto r=ji::residual(pi,i,j,g);
    rotation=std::max(rotation,r.head<3>().norm());
    residual+=r.segment<3>(3).squaredNorm()/(pi.dt*pi.dt)+r.segment<3>(6).squaredNorm();count+=6;
    accelerations.push_back(i.q_WB*pi.beta/pi.dt);
  }
  if(travel<policy.min_imu_translation_m){*reason="IMU_TRANSLATION_LOW";return false;}
  if(rotation>policy.max_rotation_residual_rad){*reason="ROTATION_RESIDUAL";return false;}
  if(std::sqrt(residual/count)>policy.max_residual_mps){*reason="ALIGNMENT_RESIDUAL";return false;}
  Eigen::Vector3d mean=Eigen::Vector3d::Zero();for(const auto& a:accelerations)mean+=a;
  mean/=accelerations.size();double var=0;for(const auto& a:accelerations)var+=(a-mean).squaredNorm();
  if(std::sqrt(var/accelerations.size())<policy.min_excitation_mps2){*reason="EXCITATION_LOW";return false;}
  auto l=window_.linearize(false);Eigen::MatrixXd H;Eigen::VectorXd b;
  if(!JointWindow::schur(l,&H,&b,reason))return false;
  if(l.gravity_offset==int(15*fs.size()))
    H=H.bottomRightCorner(H.rows()-6,H.cols()-6).eval();
  Eigen::VectorXd d=H.diagonal().cwiseAbs().cwiseSqrt();
  if((d.array()<1e-12).any()){*reason="DATA_UNOBSERVABLE";return false;}
  H=d.cwiseInverse().asDiagonal()*H*d.cwiseInverse().asDiagonal();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig((H+H.transpose())*.5);
  if(eig.info()!=Eigen::Success || eig.eigenvalues()[0]<=0 ||
      eig.eigenvalues()[0]/eig.eigenvalues().tail(1)[0]<policy.min_singular_ratio*policy.min_singular_ratio) {
    *reason="DATA_UNOBSERVABLE";return false;
  }
  *reason="ACCEPTED";return true;
}

bool JointRuntime::bootstrap(const svo::FrameBundle::Ptr& b,const rm27::MonoAlignment& raw) {
  corrected_=false;if(failed_ || !append(b,false))return false;
  if(window_.frames().size()<raw.policy.min_frames)return false;
  if(!seeded_ && !seedMetric(raw.solve(b->at(0)->T_imu_cam().getPosition()))) {
    rm27::event("joint_seed_setter_failed",b);return false;
  }
  const JointWindow admitted=window_;
  const auto report=window_.optimize();std::string reason=report.reason;
  bool accepted=report.success && qualify(raw,&reason);
  if(!report.success){captureFailure(admitted,b,true,true);captureFailure(window_,b,true,false);}
  rm27::event(("joint_bootstrap_"+reason).c_str(),b,{double(report.success),report.initial_cost,
      report.final_cost,window_.gravity().norm(),double(report.state_rank),double(report.state_dimension),
      double(report.active_pixels),double(report.absorbed_pixels)});
  if(!accepted){
    if(!window_.marginalizeTo(raw.policy.max_frames,&reason)) {
      failed_=true;rm27::event(("joint_marginal_"+reason).c_str(),b);
    }
    pruneOwners();return false;
  }
  ready_=true;
  rm27::event("metric_filter_commit",b,{double(b->getMinTimestampSeconds()),double(report.active_pixels),double(report.active_imu_edges)});
  return true;
}

void JointRuntime::exportGeometry() {
  for(const auto& f:window_.frames()) {
    const auto it=frames_.find(f.ns);if(it==frames_.end())continue;
    const svo::Transformation T(f.q_WB,f.p_WB);
    for(const auto& camera:it->second->frames_)camera->T_f_w_=camera->T_cam_imu()*T.inverse();
  }
  for(const auto& x:window_.landmarks()) {
    const auto it=points_.find(x.first);if(it==points_.end())continue;
    if(auto p=it->second.lock())p->pos_=x.second;
  }
}
void JointRuntime::forward(const svo::FrameBundle::Ptr& b) {
  if(!ready_ || failed_ || window_.frames().empty())return;
  const auto& f=window_.frames().back();ji::Preintegration p;std::string error;
  if(!ji::preintegrate(ji::fromSvoMeasurements(b->imu_datas_),f.timestamp,
      b->getMinTimestampSeconds(),f.ba,f.bg,calibration_,&p,&error)) {
    rm27::event(("joint_forward_"+error).c_str(),b);return;
  }
  const auto q=(f.q_WB*p.dR).normalized();
  const Eigen::Vector3d pos=f.p_WB+f.v_W*p.dt+.5*window_.gravity()*p.dt*p.dt+f.q_WB*p.alpha;
  const svo::Transformation T(q,pos);
  for(const auto& camera:b->frames_)camera->T_f_w_=camera->T_cam_imu()*T.inverse();
}
int JointRuntime::backward(const svo::FrameBundle::Ptr& b) {
  corrected_=false;if(failed_ || frames_.count(b->getMinTimestampNanoseconds()))return 0;
  const JointWindow before=window_;
  const auto reject=[&](const char* reason) {
    window_=before;frames_.erase(b->getMinTimestampNanoseconds());failed_=true;
    rm27::event(reason,b);return 0;
  };
  if(!append(b,true))return reject("joint_tracking_admission_failed");
  const JointWindow admitted=window_;
  const auto report=window_.optimize();
  if(!report.success) {
    rm27::event(("joint_tracking_solve_failed_"+report.reason).c_str(),b,
        {report.initial_cost,report.final_cost,report.max_step,double(report.iterations),
         double(report.state_rank),double(report.state_dimension)});
    captureFailure(admitted,b,false,true);captureFailure(window_,b,false,false);return reject("joint_tracking_solve_failed");
  }
  const auto policy=rm27::MonoAlignmentPolicy();
  if(std::abs(window_.gravity().norm()-policy.gravity_mps2)>policy.gravity_tolerance_mps2)return reject("joint_tracking_gravity_failed");
  for(const auto& f:window_.frames())
    if(f.ba.norm()>policy.max_accel_bias_mps2 || f.bg.norm()>policy.max_gyro_bias_rps)return reject("joint_tracking_bias_failed");
  exportGeometry();int valid=0;
  for(const auto& camera:b->frames_)for(size_t k=0;k<camera->numFeatures();++k) {
    const auto& p=camera->landmark_vec_[k];if(!p || camera->type_vec_[k]==svo::FeatureType::kOutlier)continue;
    const Eigen::Vector3d pc=camera->T_f_w_*p->pos();
    const auto obs=camera->f_vec_.col(k);
    const double error=(obs.head<2>()/obs.z()-pc.head<2>()/pc.z()).norm()*focal_;
    if(pc.z()>0 && error<=4.){++valid;p->last_structure_optim_=camera->id_;}
    else {window_.rejectObservation(b->getMinTimestampNanoseconds(),camera->getNFrameIndex(),k);camera->type_vec_[k]=svo::FeatureType::kOutlier;camera->seed_ref_vec_[k].keyframe.reset();camera->landmark_vec_[k].reset();}
  }
  // A data-supported update must change a camera/point through a real multi-view
  // factor. Numerical convergence alone is never a published visual correction.
  corrected_=valid>0 && window_.currentMultiViewObservations(b->getMinTimestampNanoseconds())>0
      && report.initial_cost>report.final_cost;
  if(corrected_)rm27::event("schur_visual_correction",b,{double(report.active_pixels),double(valid),report.initial_cost,report.final_cost});
  rm27::event("schur_backward_end",b,{double(valid),double(window_.frames().size())});
  return valid;
}
void JointRuntime::manage() {
  if(!ready_)return;std::string error;
  if(!window_.marginalizeTo(std::max(3,tracking_frames_),&error)){failed_=true;return;}
  pruneOwners();
}
void JointRuntime::pruneOwners() {
  std::set<int64_t> live;for(const auto& f:window_.frames())live.insert(f.ns);
  for(auto it=frames_.begin();it!=frames_.end();)if(!live.count(it->first))it=frames_.erase(it);else ++it;
  for(auto it=points_.begin();it!=points_.end();)if(it->second.expired())it=points_.erase(it);else ++it;
}
}
