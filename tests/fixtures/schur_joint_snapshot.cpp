#include "joint_window.h"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

using Window=schur_vins::JointWindow;
namespace imu=svo::headless::joint_imu;
namespace {
void check(bool condition,const std::string& why) {if(!condition)throw std::runtime_error(why);}
void exactYaml(const YAML::Node& a,const YAML::Node& b,const std::string& path) {
  check(a.Type()==b.Type(),"serialized node type changed: "+path);
  if(a.IsScalar()) {check(a.Scalar()==b.Scalar(),"serialized scalar changed: "+path);return;}
  if(a.IsSequence()) {
    check(a.size()==b.size(),"serialized sequence size changed: "+path);
    for(size_t k=0;k<a.size();++k)exactYaml(a[k],b[k],path+"/"+std::to_string(k));
  } else if(a.IsMap()) {
    check(a.size()==b.size(),"serialized map size changed: "+path);
    // yaml-cpp 0.5 can reorder map entries according to node addresses. Map
    // order has no graph meaning; compare every key and exact scalar instead.
    std::map<std::string,YAML::Node> left,right;
    for(const auto& kv:a)left.emplace(kv.first.as<std::string>(),kv.second);
    for(const auto& kv:b)right.emplace(kv.first.as<std::string>(),kv.second);
    check(left.size()==right.size(),"serialized map key count changed: "+path);
    for(const auto& kv:left) {
      auto it=right.find(kv.first);check(it!=right.end(),"serialized map key absent: "+path+"/"+kv.first);
      exactYaml(kv.second,it->second,path+"/"+kv.first);
    }
  }
}
Window::Options options() {
  Window::Options o;o.imu.sigma_acc=.008;o.imu.sigma_gyro=.0012;
  o.imu.sigma_ba=.1;o.imu.sigma_bg=.03;o.max_iterations=20;return o;
}
Window::Frame frame(int k) {
  Window::Frame f;f.ns=1403715886584058112LL+50000000LL*k;f.timestamp=1403715886.5840583+.05*k;
  const double t=.05*k;f.p_WB=Eigen::Vector3d(.2*t+.1*t*t,.03*t*t,0);
  f.v_W=Eigen::Vector3d(.2+.2*t,.06*t,0);return f;
}
Window::Measurements measurements(int k) {
  Window::Measurements raw;
  for(int j=0;j<=10;++j) {imu::Sample s;s.timestamp=frame(k-1).timestamp+.005*j;
    s.linear_acceleration=Eigen::Vector3d(.2,.06,9.81007);raw.push_back(s);}
  raw.front().timestamp=frame(k-1).timestamp;raw.back().timestamp=frame(k).timestamp;return raw;
}
Eigen::Vector3d point(int i) {return Eigen::Vector3d((i%4-1.5)*.4,(i/4-1)*.4,3.5+.1*(i%3));}
Window::Observation observation(int k,int i) {
  Window::Observation o;o.frame_ns=frame(k).ns;o.point_id=i;o.feature_id=i+100;
  o.bearing=(point(i)-frame(k).p_WB).normalized();o.focal=Eigen::Vector2d(400,402);
  o.sigma_pixels=1;o.level=i%3;return o;
}
void populate(Window* w) {
  std::string error;
  for(int k=0;k<6;++k) {
    auto f=frame(k);if(k)f.p_WB+=Eigen::Vector3d(.0001*k,-.0002*k,.00005*k);
    check(w->addFrame(f,k?measurements(k):Window::Measurements(),&error),error);
  }
  check(w->setInitialPrior(frame(0).v_W,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),2,.1,.01,&error),error);
  check(w->setGravity(Eigen::Vector3d(.001,-.002,-9.805),&error),error);
  for(int i=0;i<12;++i) {
    check(w->addLandmark(i,point(i)+Eigen::Vector3d(.001,-.001,.003),&error),error);
    for(int k=i<6?0:3;k<6;++k)check(w->addObservation(observation(k,i),&error),error);
  }
}
uint64_t scalarBits(double value) {
  uint64_t bits=0;std::memcpy(&bits,&value,sizeof(value));return bits;
}
template<class A,class B> void exactVectorBits(const A& a,const B& b,const std::string& name) {
  check(a.size()==b.size(),name+" size");
  for(int k=0;k<a.size();++k)check(scalarBits(a(k))==scalarBits(b(k)),name+" source scalar bits");
}
void publicInputBits(const Window& a,const Window& b) {
  check(a.frames().size()==b.frames().size(),"source frame count");
  for(size_t k=0;k<a.frames().size();++k) {
    const auto& p=a.frames()[k];const auto& q=b.frames()[k];
    check(p.ns==q.ns && scalarBits(p.timestamp)==scalarBits(q.timestamp),"source camera timestamp bits");
    exactVectorBits(p.q_WB.coeffs(),q.q_WB.coeffs(),"source q");
    exactVectorBits(p.p_WB,q.p_WB,"source p");exactVectorBits(p.v_W,q.v_W,"source v");
    exactVectorBits(p.ba,q.ba,"source ba");exactVectorBits(p.bg,q.bg,"source bg");
  }
  exactVectorBits(a.gravity(),b.gravity(),"source gravity");
  for(const auto& p:a.landmarks())exactVectorBits(p.second,b.landmarks().at(p.first),"source landmark");
  check(a.observations().size()==b.observations().size(),"source observation count");
  for(size_t k=0;k<a.observations().size();++k) {
    const auto& p=a.observations()[k];const auto& q=b.observations()[k];
    check(p.frame_ns==q.frame_ns && p.point_id==q.point_id && p.camera_id==q.camera_id &&
          p.feature_id==q.feature_id && p.level==q.level,"source observation metadata");
    exactVectorBits(p.bearing,q.bearing,"source bearing");exactVectorBits(p.focal,q.focal,"source focal");
    exactVectorBits(p.q_CB.coeffs(),q.q_CB.coeffs(),"source extrinsic q");exactVectorBits(p.t_CB,q.t_CB,"source extrinsic p");
    check(scalarBits(p.sigma_pixels)==scalarBits(q.sigma_pixels),"source observation sigma bits");
  }
}
void equivalent(const Window& a,const Window& b) {
  const auto x=a.linearize(),y=b.linearize();
  check(x.finite==y.finite && x.reason==y.reason,"roundtrip linearization status");
  if(x.finite) {
    check(x.H.rows()==y.H.rows() && (x.H-y.H).norm()<1e-13*(1+x.H.norm()),"roundtrip H");
    check((x.b-y.b).norm()<1e-13*(1+x.b.norm()),"roundtrip b");
    check(std::abs(x.cost-y.cost)<1e-13*(1+std::abs(x.cost)),"roundtrip cost");
    check(x.points.size()==y.points.size(),"roundtrip point count");
    for(const auto& p:x.points) {
      const auto& q=y.points.at(p.first);
      check((p.second.H-q.H).norm()<1e-13*(1+p.second.H.norm()) &&
            (p.second.W-q.W).norm()<1e-13*(1+p.second.W.norm()) &&
            (p.second.b-q.b).norm()<1e-13*(1+p.second.b.norm()),"roundtrip point blocks");
    }
  }
  check(a.frames().size()==b.frames().size() && a.landmarks().size()==b.landmarks().size(),"roundtrip mean counts");
  for(size_t k=0;k<a.frames().size();++k) {
    const auto& p=a.frames()[k];const auto& q=b.frames()[k];
    check(p.ns==q.ns && p.timestamp==q.timestamp &&
          (p.q_WB.coeffs()-q.q_WB.coeffs()).norm()<1e-14 &&
          (p.p_WB-q.p_WB).norm()<1e-14 && (p.v_W-q.v_W).norm()<1e-14 &&
          (p.ba-q.ba).norm()<1e-14 && (p.bg-q.bg).norm()<1e-14,"roundtrip frame mean");
  }
  for(const auto& p:a.landmarks())check((p.second-b.landmarks().at(p.first)).norm()<1e-14,"roundtrip point mean");
  check((a.gravity()-b.gravity()).norm()<1e-14,"roundtrip gravity mean");
  const auto p=a.status(),q=b.status();
  check(p.active_pixels==q.active_pixels && p.absorbed_pixels==q.absorbed_pixels &&
        p.dropped_pixels==q.dropped_pixels && p.pending_pixels==q.pending_pixels &&
        p.pending_points==q.pending_points && p.spatial_ineligible_points==q.spatial_ineligible_points && p.active_imu_edges==q.active_imu_edges &&
        p.absorbed_imu_edges==q.absorbed_imu_edges,"roundtrip ownership counters");
}
void roundTrip(const Window& original,const std::string& directory,const std::string& name) {
  const std::string path=directory+"/"+name+".yaml",again=directory+"/"+name+"-loaded.yaml";
  const auto repeat_a=original.linearize(),repeat_b=original.linearize();
  check(repeat_a.finite && repeat_b.finite &&
        (repeat_a.H-repeat_b.H).norm()<1e-13*(1+repeat_a.H.norm()),"same-source repeated H changed");
  std::cout<<name<<" repeated_source_H_relative="<<(repeat_a.H-repeat_b.H).norm()/(1+repeat_a.H.norm())<<" ";
  std::string error;check(original.writeSnapshot(path,&error),error);
  check(!original.writeSnapshot(path,&error),"snapshot overwrote existing evidence");
  auto loaded=Window::readSnapshot(path,&error);check(bool(loaded),error);
  publicInputBits(original,*loaded);equivalent(original,*loaded);
  check(loaded->writeSnapshot(again,&error),error);
  exactYaml(YAML::LoadFile(path),YAML::LoadFile(again),"snapshot");
  Window first=original,second=*loaded;
  const auto a=first.optimize(),b=second.optimize();
  check(a.success==b.success && a.reason==b.reason && a.iterations==b.iterations &&
        std::abs(a.final_cost-b.final_cost)<1e-13*(1+std::abs(a.final_cost)),"roundtrip nonlinear solve");
  equivalent(first,second);
  ::unlink(path.c_str());::unlink(again.c_str());
}
void synthetic() {
  char temp[]="/tmp/rm27-joint-snapshot-XXXXXX";const char* created=::mkdtemp(temp);
  check(created!=nullptr,"snapshot fixture temporary directory");const std::string directory(created);
  Window w(options());populate(&w);
  roundTrip(w,directory,"initial-prior");
  std::string error;
  check(w.rejectObservation(frame(5).ns,0,106,&error),error);
  check(w.marginalizeTo(3,&error),error);
  check(w.status().absorbed_pixels==36 && w.status().dropped_pixels==1,"fixture prior/registry setup");
  roundTrip(w,directory,"marginal-prior");
  check(w.addLandmark(0,point(0),&error),error);
  check(w.addFrame(frame(6),measurements(6),&error),error);
  check(w.addObservation(observation(6,0),&error),error);
  const std::string path=directory+"/fresh-segment.yaml";
  check(w.writeSnapshot(path,&error),error);auto loaded=Window::readSnapshot(path,&error);check(bool(loaded),error);
  check(!loaded->addObservation(observation(5,0),&error),"absorbed identity lost during roundtrip");
  check(!loaded->addObservation(observation(5,6),&error),"dropped identity lost during roundtrip");
  auto old_segment=observation(5,0);old_segment.feature_id+=100000;
  check(!loaded->addObservation(old_segment,&error),"retirement cutoff lost during roundtrip");
  check(!loaded->addFrame(frame(6),measurements(6),&error),"IMU/frame ownership lost during roundtrip");
  equivalent(w,*loaded);::unlink(path.c_str());::rmdir(directory.c_str());
  std::cout<<"snapshot_roundtrip_H_b_cost_registries_nonlinear PASS\n";
}
void actual(const std::string& path,int passes) {
  std::string error;auto w=Window::readSnapshot(path,&error);check(bool(w),error);
  const auto ownership=w->status();
  auto initial=w->linearize();
  std::cout<<"captured_graph finite="<<initial.finite<<" reason="<<initial.reason
           <<" cost="<<initial.cost<<" frames="<<w->frames().size()
           <<" points="<<ownership.active_points<<" pixels="<<ownership.active_pixels
           <<" absorbed="<<ownership.absorbed_pixels<<" dropped="<<ownership.dropped_pixels
           <<" pending_pixels="<<ownership.pending_pixels<<" pending_points="<<ownership.pending_points
           <<" spatial_ineligible_points="<<ownership.spatial_ineligible_points<<"\n";
  Window deterministic=*w;
  for(int pass=0;pass<passes;++pass) {
    const auto r=w->optimize(),current=w->status();
    check(current.active_pixels==ownership.active_pixels && current.absorbed_pixels==ownership.absorbed_pixels &&
          current.dropped_pixels==ownership.dropped_pixels && current.pending_pixels==ownership.pending_pixels &&
          current.pending_points==ownership.pending_points && current.active_imu_edges==ownership.active_imu_edges &&
          current.absorbed_imu_edges==ownership.absorbed_imu_edges,"replay changed raw measurement ownership");
    if(pass==0) {
      const auto second=deterministic.optimize();
      check(r.success==second.success && r.reason==second.reason && r.iterations==second.iterations &&
            std::abs(r.final_cost-second.final_cost)<1e-12*(1+std::abs(r.final_cost)),"captured first solve not deterministic");
    }
    std::cout<<"same_graph_pass="<<pass+1<<" success="<<r.success<<" finite="<<r.finite
             <<" iterations="<<r.iterations<<" initial_cost="<<r.initial_cost<<" final_cost="<<r.final_cost
             <<" state_rank="<<r.state_rank<<" state_dimension="<<r.state_dimension
             <<" reason="<<r.reason<<"\n";
  }
  std::cout<<"REPLAY_COMPLETE (numerical diagnostic; no VIO acceptance claim)\n";
}
}
int main(int argc,char** argv) {
  std::cout<<std::setprecision(17);
  try {
    if(argc==1)synthetic();
    else {int passes=argc>2?std::stoi(argv[2]):4;check(passes>=1&&passes<=20,"passes must be 1..20");actual(argv[1],passes);}
    return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
