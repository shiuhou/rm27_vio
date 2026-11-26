#include "joint_window.h"
#include <yaml-cpp/yaml.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <limits>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

namespace schur_vins {
namespace {
using Mat=Eigen::MatrixXd;
using Vec=Eigen::VectorXd;
using Node=YAML::Node;
Node sequence() { return Node(YAML::NodeType::Sequence); }
// yaml-cpp 0.5 in the production image encodes Node(double) with only
// digits10+1=16 digits BEFORE Emitter precision applies. Camera epoch seconds
// then lose endpoint bits and change the raw IMU interval. Encode ourselves.
Node scalar(double value) {
  if(std::isnan(value))return Node(".nan");
  if(std::isinf(value))return Node(value<0?"-.inf":".inf");
  std::ostringstream text;text.imbue(std::locale::classic());
  text<<std::setprecision(std::numeric_limits<double>::max_digits10)<<value;
  Node encoded(text.str());
  const double decoded=encoded.as<double>();
  uint64_t before=0,after=0;
  std::memcpy(&before,&value,sizeof(value));std::memcpy(&after,&decoded,sizeof(decoded));
  if(before!=after)throw std::runtime_error("snapshot scalar changed source double bits");
  return encoded;
}
template<class Derived> Node vectorNode(const Eigen::MatrixBase<Derived>& v) {
  Node node=sequence();for(int i=0;i<v.size();++i)node.push_back(scalar(v(i)));return node;
}
Vec readVector(const Node& node,int size) {
  if(!node.IsSequence()||int(node.size())!=size)throw std::runtime_error("vector shape mismatch");
  Vec v(size);for(int i=0;i<size;++i)v[i]=node[i].as<double>();return v;
}
Node matrixNode(const Mat& m) {
  Node node;node["rows"]=m.rows();node["cols"]=m.cols();node["data"]=sequence();
  for(int r=0;r<m.rows();++r)for(int c=0;c<m.cols();++c)node["data"].push_back(scalar(m(r,c)));
  return node;
}
Mat readMatrix(const Node& node) {
  const int rows=node["rows"].as<int>(),cols=node["cols"].as<int>();
  if(rows<0||cols<0||!node["data"].IsSequence()||uint64_t(rows)*uint64_t(cols)!=node["data"].size())
    throw std::runtime_error("matrix shape mismatch");
  Mat m(rows,cols);
  for(int r=0;r<rows;++r)for(int c=0;c<cols;++c)m(r,c)=node["data"][r*cols+c].as<double>();
  return m;
}
Node quaternionNode(const Eigen::Quaterniond& q) {
  Node node=sequence();node.push_back(scalar(q.w()));node.push_back(scalar(q.x()));node.push_back(scalar(q.y()));node.push_back(scalar(q.z()));return node;
}
Eigen::Quaterniond readQuaternion(const Node& node) {
  Vec q=readVector(node,4);return Eigen::Quaterniond(q[0],q[1],q[2],q[3]);
}
Node frameNode(const JointWindow::Frame& f) {
  Node n;n["ns"]=f.ns;n["timestamp"]=scalar(f.timestamp);n["q_WB"]=quaternionNode(f.q_WB);
  n["p_WB"]=vectorNode(f.p_WB);n["v_W"]=vectorNode(f.v_W);
  n["ba"]=vectorNode(f.ba);n["bg"]=vectorNode(f.bg);return n;
}
JointWindow::Frame readFrame(const Node& n) {
  JointWindow::Frame f;f.ns=n["ns"].as<int64_t>();f.timestamp=n["timestamp"].as<double>();
  f.q_WB=readQuaternion(n["q_WB"]);f.p_WB=readVector(n["p_WB"],3);
  f.v_W=readVector(n["v_W"],3);f.ba=readVector(n["ba"],3);f.bg=readVector(n["bg"],3);return f;
}
Node framesNode(const std::vector<JointWindow::Frame>& frames) {
  Node n=sequence();for(const auto& f:frames)n.push_back(frameNode(f));return n;
}
std::vector<JointWindow::Frame> readFrames(const Node& node) {
  if(!node.IsSequence())throw std::runtime_error("frames must be a sequence");
  std::vector<JointWindow::Frame> result;
  for(const auto& n:node)result.push_back(readFrame(n));return result;
}
Node reportNode(const JointWindow::SolveReport& r) {
  Node n;n["success"]=r.success;n["finite"]=r.finite;n["iterations"]=r.iterations;
  n["state_dimension"]=r.state_dimension;n["state_rank"]=r.state_rank;
  n["initial_cost"]=scalar(r.initial_cost);n["final_cost"]=scalar(r.final_cost);n["max_step"]=scalar(r.max_step);
  n["spatial_ineligible_points"]=r.spatial_ineligible_points;
  n["pending_pixels"]=r.pending_pixels;n["pending_points"]=r.pending_points;
  n["active_pixels"]=r.active_pixels;n["absorbed_pixels"]=r.absorbed_pixels;n["dropped_pixels"]=r.dropped_pixels;
  n["active_imu_edges"]=r.active_imu_edges;n["absorbed_imu_edges"]=r.absorbed_imu_edges;
  n["active_points"]=r.active_points;n["reason"]=r.reason;return n;
}
JointWindow::SolveReport readReport(const Node& n) {
  JointWindow::SolveReport r;
  r.spatial_ineligible_points=n["spatial_ineligible_points"]?n["spatial_ineligible_points"].as<size_t>():0;
  r.pending_pixels=n["pending_pixels"]?n["pending_pixels"].as<size_t>():0;
  r.pending_points=n["pending_points"]?n["pending_points"].as<size_t>():0;
  r.success=n["success"].as<bool>();r.finite=n["finite"].as<bool>();
  r.iterations=n["iterations"].as<int>();r.state_dimension=n["state_dimension"].as<int>();r.state_rank=n["state_rank"].as<int>();
  r.initial_cost=n["initial_cost"].as<double>();r.final_cost=n["final_cost"].as<double>();r.max_step=n["max_step"].as<double>();
  r.active_pixels=n["active_pixels"].as<size_t>();r.absorbed_pixels=n["absorbed_pixels"].as<size_t>();r.dropped_pixels=n["dropped_pixels"].as<size_t>();
  r.active_imu_edges=n["active_imu_edges"].as<size_t>();r.absorbed_imu_edges=n["absorbed_imu_edges"].as<size_t>();
  r.active_points=n["active_points"].as<size_t>();r.reason=n["reason"].as<std::string>();return r;
}
bool fail(std::string* error,const std::string& why) {if(error)*error=why;return false;}
}

bool JointWindow::writeSnapshot(const std::string& path,std::string* error) const {
  try {
    Node root;root["schema"]="rm27_joint_window";root["version"]=2;root["floating_point_encoding"]="max_digits10_checked";
    auto o=root["options"];
    o["huber_pixels"]=scalar(options_.huber_pixels);o["max_iterations"]=options_.max_iterations;
    o["sigma_acc"]=scalar(options_.imu.sigma_acc);o["sigma_gyro"]=scalar(options_.imu.sigma_gyro);
    o["sigma_ba"]=scalar(options_.imu.sigma_ba);o["sigma_bg"]=scalar(options_.imu.sigma_bg);o["max_gap"]=scalar(options_.imu.max_gap);
    root["frames"]=framesNode(frames_);root["gravity"]=vectorNode(gravity_);
    root["gauge_ns"]=gauge_ns_;root["gauge"]=frameNode(gauge_);
    root["points"]=sequence();
    for(const auto& p:points_) {Node n;n["id"]=p.first;n["position"]=vectorNode(p.second);root["points"].push_back(n);}
    root["observations"]=sequence();
    for(const auto& p:observations_) {
      Node n;n["frame_ns"]=p.frame_ns;n["point_id"]=p.point_id;n["camera_id"]=p.camera_id;n["feature_id"]=p.feature_id;
      n["bearing"]=vectorNode(p.bearing);n["q_CB"]=quaternionNode(p.q_CB);n["t_CB"]=vectorNode(p.t_CB);
      n["focal"]=vectorNode(p.focal);n["sigma_pixels"]=scalar(p.sigma_pixels);n["level"]=p.level;root["observations"].push_back(n);
    }
    root["edges"]=sequence();
    for(const auto& e:edges_) {
      Node n;n["first"]=e.key.first;n["second"]=e.key.second;n["raw"]=sequence();
      for(const auto& s:e.raw) {Node sample;sample["timestamp"]=scalar(s.timestamp);
        sample["angular_velocity"]=vectorNode(s.angular_velocity);sample["linear_acceleration"]=vectorNode(s.linear_acceleration);
        n["raw"].push_back(sample);}
      root["edges"].push_back(n);
    }
    auto initial=root["initial"];initial["active"]=initial_.active;
    if(initial_.active) {
      initial["v"]=vectorNode(initial_.v);initial["ba"]=vectorNode(initial_.ba);initial["bg"]=vectorNode(initial_.bg);
      initial["sigma_v"]=scalar(initial_.sigma_v);initial["sigma_ba"]=scalar(initial_.sigma_ba);initial["sigma_bg"]=scalar(initial_.sigma_bg);
    }
    auto prior=root["prior"];prior["A"]=matrixNode(prior_.A);prior["rhs"]=vectorNode(prior_.rhs);
    prior["anchors"]=framesNode(prior_.anchors);
    // Default inactive prior has no initialized gravity; never read its bytes.
    prior["gravity"]=vectorNode(prior_.anchors.empty()?Eigen::Vector3d::Zero():prior_.gravity);
    auto pixelRegistry=[&](const char* name,const std::set<PixelKey>& keys) {
      root[name]=sequence();for(const auto& k:keys) {Node n=sequence();n.push_back(std::get<0>(k));n.push_back(std::get<1>(k));n.push_back(std::get<2>(k));root[name].push_back(n);}
    };
    pixelRegistry("active_pixels",active_pixels_);pixelRegistry("absorbed_pixels",absorbed_pixels_);pixelRegistry("dropped_pixels",dropped_pixels_);
    auto edgeRegistry=[&](const char* name,const std::set<EdgeKey>& keys) {
      root[name]=sequence();for(const auto& k:keys) {Node n=sequence();n.push_back(k.first);n.push_back(k.second);root[name].push_back(n);}
    };
    edgeRegistry("active_edges",active_edges_);edgeRegistry("absorbed_edges",absorbed_edges_);
    root["retired_after"]=sequence();for(const auto& p:retired_after_) {Node n=sequence();n.push_back(p.first);n.push_back(p.second);root["retired_after"].push_back(n);}
    root["report"]=reportNode(report_);
    YAML::Emitter emitter;emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);emitter<<root;
    if(!emitter.good())return fail(error,"snapshot YAML emission failed");
    const int fd=::open(path.c_str(),O_WRONLY|O_CREAT|O_EXCL,0644);
    if(fd<0)return fail(error,"snapshot open: "+std::string(std::strerror(errno)));
    const char* data=emitter.c_str();size_t remaining=emitter.size();
    while(remaining) {
      const ssize_t bytes=::write(fd,data,remaining);
      if(bytes<0 && errno==EINTR)continue;
      if(bytes<=0) {const std::string message=std::strerror(errno);::close(fd);::unlink(path.c_str());return fail(error,"snapshot write: "+message);}
      data+=bytes;remaining-=size_t(bytes);
    }
    if(::close(fd)!=0) {::unlink(path.c_str());return fail(error,"snapshot close failed");}
    return true;
  } catch(const std::exception& e) {return fail(error,"snapshot write: "+std::string(e.what()));}
}

std::unique_ptr<JointWindow> JointWindow::readSnapshot(const std::string& path,std::string* error) {
  try {
    const Node root=YAML::LoadFile(path);
    const int version=root["version"].as<int>();
    if(root["schema"].as<std::string>()!="rm27_joint_window"||(version!=1&&version!=2))
      throw std::runtime_error("unsupported snapshot schema/version");
    if(version==2 && root["floating_point_encoding"].as<std::string>()!="max_digits10_checked")
      throw std::runtime_error("unsupported snapshot floating point encoding");
    Options options;const auto o=root["options"];
    options.huber_pixels=o["huber_pixels"].as<double>();options.max_iterations=o["max_iterations"].as<int>();
    options.imu.sigma_acc=o["sigma_acc"].as<double>();options.imu.sigma_gyro=o["sigma_gyro"].as<double>();
    options.imu.sigma_ba=o["sigma_ba"].as<double>();options.imu.sigma_bg=o["sigma_bg"].as<double>();options.imu.max_gap=o["max_gap"].as<double>();
    std::unique_ptr<JointWindow> w(new JointWindow(options));
    w->frames_=readFrames(root["frames"]);w->gravity_=readVector(root["gravity"],3);
    w->gauge_ns_=root["gauge_ns"].as<int64_t>();w->gauge_=readFrame(root["gauge"]);
    for(const auto& n:root["points"])if(!w->points_.emplace(n["id"].as<int64_t>(),readVector(n["position"],3)).second)
      throw std::runtime_error("duplicate landmark ID");
    for(const auto& n:root["observations"]) {
      Observation p;p.frame_ns=n["frame_ns"].as<int64_t>();p.point_id=n["point_id"].as<int64_t>();
      p.camera_id=n["camera_id"].as<int>();p.feature_id=n["feature_id"].as<uint64_t>();
      p.bearing=readVector(n["bearing"],3);p.q_CB=readQuaternion(n["q_CB"]);p.t_CB=readVector(n["t_CB"],3);
      p.focal=readVector(n["focal"],2);p.sigma_pixels=n["sigma_pixels"].as<double>();p.level=n["level"].as<int>();w->observations_.push_back(p);
    }
    for(const auto& n:root["edges"]) {
      Edge e;e.key={n["first"].as<int64_t>(),n["second"].as<int64_t>()};
      for(const auto& s:n["raw"]) {svo::headless::joint_imu::Sample sample;sample.timestamp=s["timestamp"].as<double>();
        sample.angular_velocity=readVector(s["angular_velocity"],3);sample.linear_acceleration=readVector(s["linear_acceleration"],3);e.raw.push_back(sample);}
      w->edges_.push_back(std::move(e));
    }
    const auto initial=root["initial"];w->initial_.active=initial["active"].as<bool>();
    if(w->initial_.active) {
      w->initial_.v=readVector(initial["v"],3);w->initial_.ba=readVector(initial["ba"],3);w->initial_.bg=readVector(initial["bg"],3);
      w->initial_.sigma_v=initial["sigma_v"].as<double>();w->initial_.sigma_ba=initial["sigma_ba"].as<double>();w->initial_.sigma_bg=initial["sigma_bg"].as<double>();
    }
    const auto prior=root["prior"];w->prior_.A=readMatrix(prior["A"]);w->prior_.rhs=readVector(prior["rhs"],w->prior_.A.rows());
    w->prior_.anchors=readFrames(prior["anchors"]);w->prior_.gravity=readVector(prior["gravity"],3);
    auto pixels=[&](const char* name,std::set<PixelKey>* keys) {
      for(const auto& n:root[name]) {if(!n.IsSequence()||n.size()!=3)throw std::runtime_error("pixel registry shape");
        if(!keys->emplace(n[0].as<int64_t>(),n[1].as<int>(),n[2].as<uint64_t>()).second)throw std::runtime_error("duplicate pixel registry ID");}
    };
    pixels("active_pixels",&w->active_pixels_);pixels("absorbed_pixels",&w->absorbed_pixels_);pixels("dropped_pixels",&w->dropped_pixels_);
    auto edges=[&](const char* name,std::set<EdgeKey>* keys) {
      for(const auto& n:root[name]) {if(!n.IsSequence()||n.size()!=2)throw std::runtime_error("edge registry shape");
        if(!keys->emplace(n[0].as<int64_t>(),n[1].as<int64_t>()).second)throw std::runtime_error("duplicate edge registry ID");}
    };
    edges("active_edges",&w->active_edges_);edges("absorbed_edges",&w->absorbed_edges_);
    for(const auto& n:root["retired_after"]) {
      if(!n.IsSequence()||n.size()!=2||!w->retired_after_.emplace(n[0].as<int64_t>(),n[1].as<int64_t>()).second)
        throw std::runtime_error("retired point registry shape/identity");
    }
    w->report_=readReport(root["report"]);
    std::set<int64_t> frames;
    for(const auto& f:w->frames_)if(!frames.insert(f.ns).second)throw std::runtime_error("duplicate frame ID");
    std::set<PixelKey> observed;
    for(const auto& p:w->observations_) {
      const PixelKey key(p.frame_ns,p.camera_id,p.feature_id);
      if(!frames.count(p.frame_ns)||!w->points_.count(p.point_id)||!observed.insert(key).second)
        throw std::runtime_error("observation graph reference/identity");
    }
    if(observed!=w->active_pixels_)throw std::runtime_error("active pixel registry differs from raw factors");
    for(const auto& k:w->active_pixels_)if(w->absorbed_pixels_.count(k)||w->dropped_pixels_.count(k))
      throw std::runtime_error("pixel registry ownership overlap");
    for(const auto& k:w->absorbed_pixels_)if(w->dropped_pixels_.count(k))throw std::runtime_error("retired pixel ownership overlap");
    std::set<EdgeKey> represented;
    for(const auto& e:w->edges_)if(!frames.count(e.key.first)||!frames.count(e.key.second)||!represented.insert(e.key).second)
      throw std::runtime_error("IMU graph reference/identity");
    if(represented!=w->active_edges_)throw std::runtime_error("active IMU registry differs from raw factors");
    for(const auto& k:w->active_edges_)if(w->absorbed_edges_.count(k))throw std::runtime_error("IMU registry ownership overlap");
    int prior_dimension=3;
    for(const auto& f:w->prior_.anchors) {
      if(!frames.count(f.ns))throw std::runtime_error("prior anchor absent from graph");
      prior_dimension+=f.ns==w->gauge_ns_?9:15;
    }
    if(w->prior_.A.cols()!=0&&w->prior_.A.cols()!=prior_dimension)throw std::runtime_error("prior tangent dimension mismatch");
    // Do not normalize, re-seed, re-admit or re-integrate while loading: even
    // invalid numerical means must survive to reproduce the captured failure.
    return w;
  } catch(const std::exception& e) {fail(error,"snapshot read: "+std::string(e.what()));return nullptr;}
}
}  // namespace schur_vins
