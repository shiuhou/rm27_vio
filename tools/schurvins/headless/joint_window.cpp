#include "joint_window.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>

namespace schur_vins {
namespace {
namespace imu = svo::headless::joint_imu;
using Mat = Eigen::MatrixXd;
using Vec = Eigen::VectorXd;
constexpr size_t kMinObservations = 3;  // Preserved SchurVINS MIN_OBS contract.
constexpr double kMaxPointNorm = 1e3;  // Preserved SchurVINS MAX_DIST contract.
bool fail(std::string* error, const std::string& message) {
  if (error) *error = message;
  return false;
}
Eigen::Matrix3d skew(const Eigen::Vector3d& x) {
  Eigen::Matrix3d a;
  a << 0,-x.z(),x.y(), x.z(),0,-x.x(), -x.y(),x.x(),0;
  return a;
}
Eigen::Matrix3d rightLogJacobian(const Eigen::Vector3d& x) {
  const double t2 = x.squaredNorm();
  double a = 1.0 / 12.0;
  if (t2 > 1e-8) {
    const double t = std::sqrt(t2);
    a = (1.0 - 0.5*t/std::tan(0.5*t)) / t2;
  } else {
    a += t2 / 720.0;
  }
  return Eigen::Matrix3d::Identity() + 0.5*skew(x) + a*skew(x)*skew(x);
}
// Numerical rank is relative to roundoff, not a physical observability gate.
// No nullspace eigenvalue is replaced with positive information.
bool psdInverse(const Mat& input, Mat* inverse, int* rank,
                std::string* error = nullptr) {
  if (input.rows() == 0) {
    inverse->resize(0,0); if (rank) *rank=0; return true;
  }
  if (!input.allFinite() || input.rows()!=input.cols())
    return fail(error,"nonfinite/non-square information");
  Eigen::SelfAdjointEigenSolver<Mat> eig((input+input.transpose())*.5);
  if (eig.info()!=Eigen::Success) return fail(error,"information eigensolve failed");
  const double scale=std::max(1.0,eig.eigenvalues().cwiseAbs().maxCoeff());
  const double tolerance=128*std::numeric_limits<double>::epsilon()*input.rows()*scale;
  if (eig.eigenvalues().minCoeff() < -32*tolerance)
    return fail(error,"indefinite information");
  Vec values=eig.eigenvalues();
  int r=0;
  for (int k=0;k<values.size();++k) {
    if (values[k]>tolerance) {values[k]=1/values[k];++r;} else values[k]=0;
  }
  *inverse=eig.eigenvectors()*values.asDiagonal()*eig.eigenvectors().transpose();
  if (rank) *rank=r;
  return true;
}
bool validFrame(const JointWindow::Frame& f) {
  return f.ns>0 && std::isfinite(f.timestamp) && f.timestamp>=0 && f.q_WB.coeffs().allFinite() &&
      std::abs(f.q_WB.norm()-1)<1e-6 && f.p_WB.allFinite() &&
      f.v_W.allFinite() && f.ba.allFinite() && f.bg.allFinite();
}
double seconds(const JointWindow::Frame& f) {
  return f.timestamp>0?f.timestamp:double(f.ns)*1e-9;
}
imu::State toImu(const JointWindow::Frame& f) {
  imu::State s;
  s.timestamp=seconds(f); s.q_WB=f.q_WB;
  s.p_WB=f.p_WB; s.v_W=f.v_W; s.ba=f.ba; s.bg=f.bg;
  return s;
}
void perturb(JointWindow::Frame* f,int component,double d) {
  if (component<3) {
    Eigen::Vector3d w=Eigen::Vector3d::Zero();w[component]=d;
    f->q_WB=(f->q_WB*imu::expSO3(w)).normalized();
  } else if(component<6) f->p_WB[component-3]+=d;
  else if(component<9) f->v_W[component-6]+=d;
  else if(component<12) f->ba[component-9]+=d;
  else f->bg[component-12]+=d;
}
}  // namespace

JointWindow::JointWindow(const Options& options):options_(options) {}

bool JointWindow::addFrame(const Frame& f,const Measurements& raw,std::string* error) {
  if(!validFrame(f)) return fail(error,"invalid frame mean");
  if(!frames_.empty() && (f.ns<=frames_.back().ns || seconds(f)<=seconds(frames_.back())))
    return fail(error,"frame timestamp must be new and strictly increasing");
  if(frames_.empty()) {
    if(!raw.empty()) return fail(error,"first frame cannot consume an IMU edge");
    gauge_ns_=f.ns;gauge_=f;
  } else {
    EdgeKey key(frames_.back().ns,f.ns);
    if(active_edges_.count(key)||absorbed_edges_.count(key))
      return fail(error,"IMU edge already represented");
    imu::Preintegration pi;
    if(!imu::preintegrate(raw,seconds(frames_.back()),seconds(f),
                         frames_.back().ba,frames_.back().bg,options_.imu,&pi,error)) return false;
    edges_.push_back({key,raw});active_edges_.insert(key);
  }
  frames_.push_back(f);return true;
}
bool JointWindow::addLandmark(int64_t id,const Eigen::Vector3d& p,std::string* error) {
  if(!p.allFinite()) return fail(error,"nonfinite landmark seed");
  if(!points_.count(id)) points_.emplace(id,p);
  return true;
}
bool JointWindow::addObservation(const Observation& o,std::string* error) {
  const PixelKey key(o.frame_ns,o.camera_id,o.feature_id);
  if(active_pixels_.count(key)||absorbed_pixels_.count(key)||dropped_pixels_.count(key))
    return fail(error,"pixel factor already represented");
  if(!points_.count(o.point_id)) return fail(error,"observation needs free active landmark");
  if(std::none_of(frames_.begin(),frames_.end(),[&](const Frame& f){return f.ns==o.frame_ns;}))
    return fail(error,"observation frame is not active");
  auto retired=retired_after_.find(o.point_id);
  if(retired!=retired_after_.end() && o.frame_ns<=retired->second)
    return fail(error,"retired segment only accepts future frame measurements");
  if(!o.bearing.allFinite() || o.bearing.z()<=0 || !o.focal.allFinite() ||
     o.focal.minCoeff()<=0 || !std::isfinite(o.sigma_pixels) || o.sigma_pixels<=0 ||
     !o.q_CB.coeffs().allFinite() || std::abs(o.q_CB.norm()-1)>1e-6 ||
     !o.t_CB.allFinite() || o.camera_id<0 || o.level<0)
    return fail(error,"invalid calibrated pixel factor");
  observations_.push_back(o);active_pixels_.insert(key);return true;
}
bool JointWindow::rejectObservation(int64_t ns,int camera_id,uint64_t feature_id,
    std::string* error) {
  const PixelKey key(ns,camera_id,feature_id);
  if(!active_pixels_.count(key)) return fail(error,"pixel factor is not active");
  auto it=std::find_if(observations_.begin(),observations_.end(),[&](const Observation& o){
    return PixelKey(o.frame_ns,o.camera_id,o.feature_id)==key;
  });
  if(it==observations_.end()) return fail(error,"active pixel registry is inconsistent");
  const int64_t point_id=it->point_id;
  observations_.erase(it);active_pixels_.erase(key);dropped_pixels_.insert(key);
  if(std::none_of(observations_.begin(),observations_.end(),[&](const Observation& o){
       return o.point_id==point_id;
     })) {
    points_.erase(point_id);
    if(!frames_.empty()) retired_after_[point_id]=frames_.back().ns;
  }
  return true;
}
size_t JointWindow::currentMultiViewObservations(int64_t ns) const {
  if(ns==gauge_ns_ || !std::isfinite(options_.huber_pixels) || options_.huber_pixels<=0)
    return 0;
  std::map<int64_t,const Frame*> by_ns;
  for(const auto& f:frames_) by_ns[f.ns]=&f;
  if(!by_ns.count(ns)) return 0;
  struct VisualBlock {
    Eigen::Matrix3d H=Eigen::Matrix3d::Zero();
    Eigen::Matrix<double,6,6> Hcc=Eigen::Matrix<double,6,6>::Zero();
    Eigen::Matrix<double,6,3> Wc=Eigen::Matrix<double,6,3>::Zero();
    std::set<int64_t> frame_views;
    size_t current_rows=0;
  };
  std::map<int64_t,VisualBlock> blocks;
  for(const auto& o:observations_) {
    const auto& f=*by_ns.at(o.frame_ns);
    const Eigen::Matrix3d Rbw=f.q_WB.toRotationMatrix().transpose();
    const Eigen::Matrix3d Rcb=o.q_CB.toRotationMatrix();
    const Eigen::Vector3d pb=Rbw*(points_.at(o.point_id)-f.p_WB);
    const Eigen::Vector3d pc=Rcb*pb+o.t_CB;
    if(!pc.allFinite()||pc.z()<=1e-8) continue;
    const double radius=((pc.head<2>()/pc.z()-o.bearing.head<2>()/o.bearing.z())
                          .cwiseProduct(o.focal)).norm();
    const double weight=radius>options_.huber_pixels?std::sqrt(options_.huber_pixels/radius):1;
    Eigen::Matrix<double,2,3> projection;
    projection << 1/pc.z(),0,-pc.x()/(pc.z()*pc.z()),
                  0,1/pc.z(),-pc.y()/(pc.z()*pc.z());
    projection=o.focal.asDiagonal()*projection*(weight/o.sigma_pixels);
    const Eigen::Matrix<double,2,3> Jp=projection*Rcb*Rbw;
    auto& block=blocks[o.point_id];
    block.H.noalias()+=Jp.transpose()*Jp;block.frame_views.insert(o.frame_ns);
    if(o.frame_ns==ns) {
      Eigen::Matrix<double,2,6> Jc;
      Jc.leftCols<3>()=projection*Rcb*skew(pb);Jc.rightCols<3>()=-Jp;
      block.Hcc.noalias()+=Jc.transpose()*Jc;
      block.Wc.noalias()+=Jc.transpose()*Jp;++block.current_rows;
    }
  }
  size_t count=0;
  for(const auto& pair:blocks) {
    const auto& block=pair.second;
    if(block.frame_views.size()<kMinObservations||block.current_rows==0||
       points_.at(pair.first).norm()>kMaxPointNorm) continue;
    Mat inverse;int rank=0;
    if(!psdInverse(block.H,&inverse,&rank)||rank!=3) continue;
    const Eigen::Matrix<double,6,6> reduced=block.Hcc-block.Wc*inverse*block.Wc.transpose();
    const double roundoff=128*std::numeric_limits<double>::epsilon()*6*
        std::max(1.0,block.Hcc.cwiseAbs().maxCoeff());
    if(reduced.allFinite() && reduced.trace()>roundoff) count+=block.current_rows;
  }
  return count;
}
bool JointWindow::setInitialPrior(const Eigen::Vector3d& v,const Eigen::Vector3d& ba,
    const Eigen::Vector3d& bg,double sv,double sa,double sg,std::string* error) {
  if(frames_.empty() || frames_.front().ns!=gauge_ns_ || initial_.active ||
     prior_.A.size()!=0 || report_.iterations!=0)
    return fail(error,"initial prior must be installed once before optimization");
  if(!v.allFinite()||!ba.allFinite()||!bg.allFinite()||!std::isfinite(sv)||sv<=0 ||
     !std::isfinite(sa)||sa<=0||!std::isfinite(sg)||sg<=0)
    return fail(error,"initial velocity/bias prior requires configured positive sigmas");
  initial_.active=true;initial_.v=v;initial_.ba=ba;initial_.bg=bg;
  initial_.sigma_v=sv;initial_.sigma_ba=sa;initial_.sigma_bg=sg;
  return true;
}
bool JointWindow::setFrameSeed(const Frame& f,std::string* error) {
  if(!validFrame(f)) return fail(error,"invalid frame seed");
  if(f.ns==gauge_ns_ && ((f.p_WB-gauge_.p_WB).norm()>1e-12 ||
      imu::logSO3(gauge_.q_WB.conjugate()*f.q_WB).norm()>1e-12))
    return fail(error,"first position/orientation gauge is immutable");
  for(auto& x:frames_) if(x.ns==f.ns) {
    if(seconds(x)!=seconds(f)) return fail(error,"frame seed cannot change factor endpoint time");
    x=f;return true;
  }
  return fail(error,"seed frame is not active");
}
bool JointWindow::setLandmarkSeed(int64_t id,const Eigen::Vector3d& p,std::string* error) {
  auto it=points_.find(id);
  if(it==points_.end()||!p.allFinite()) return fail(error,"invalid active landmark seed");
  it->second=p;return true;
}
bool JointWindow::setGravity(const Eigen::Vector3d& g,std::string* error) {
  if(!g.allFinite()) return fail(error,"nonfinite gravity seed");
  gravity_=g;return true;
}
std::set<int64_t> JointWindow::eligiblePoints() const {
  std::map<int64_t,std::set<int64_t>> views;
  for(const auto& o:observations_)views[o.point_id].insert(o.frame_ns);
  std::set<int64_t> mature;
  for(const auto& p:views)if(p.second.size()>=kMinObservations &&
      points_.at(p.first).allFinite() && points_.at(p.first).norm()<=kMaxPointNorm)mature.insert(p.first);
  return mature;
}
int JointWindow::column(const Linearization& l,int64_t ns,int k) const {
  auto it=l.frame_offsets.find(ns);
  if(it==l.frame_offsets.end()) return -1;
  if(ns==gauge_ns_) return k<6?-1:it->second+k-6;
  return it->second+k;
}
JointWindow::Linearization JointWindow::linearize(bool include_priors) const {
  return assemble(nullptr,nullptr,include_priors);
}
JointWindow::Linearization JointWindow::assemble(const std::set<int64_t>* removed,
    const std::set<int64_t>* absorbed,bool include_priors) const {
  Linearization l;
  if(frames_.empty()) {l.reason="empty window";return l;}
  int dim=0;
  for(const auto& f:frames_) {l.frame_offsets[f.ns]=dim;dim+=f.ns==gauge_ns_?9:15;}
  l.gravity_offset=dim;dim+=3;
  l.H=Mat::Zero(dim,dim);l.b=Vec::Zero(dim);
  std::map<int64_t,const Frame*> by_ns;
  for(const auto& f:frames_) by_ns[f.ns]=&f;
  auto add=[&](const Mat& J,const Vec& r) {
    l.H.noalias()+=J.transpose()*J;l.b.noalias()-=J.transpose()*r;
    l.cost+=.5*r.squaredNorm();
  };
  auto addSparse=[&](const Mat& J,const Vec& r) {
    std::vector<int> support;
    for(int c=0;c<J.cols();++c) if(J.col(c).squaredNorm()>0) support.push_back(c);
    Mat compact(J.rows(),support.size());
    for(size_t c=0;c<support.size();++c) compact.col(c)=J.col(support[c]);
    const Mat Hlocal=compact.transpose()*compact;
    const Vec blocal=-compact.transpose()*r;
    for(size_t i=0;i<support.size();++i) {
      l.b[support[i]]+=blocal[i];
      for(size_t j=0;j<support.size();++j) l.H(support[i],support[j])+=Hlocal(i,j);
    }
    l.cost+=.5*r.squaredNorm();
  };
  if(include_priors && prior_.A.rows()) {
    Vec delta=Vec::Zero(prior_.A.cols());
    Mat D=Mat::Zero(prior_.A.cols(),dim);
    int off=0;
    for(const auto& anchor:prior_.anchors) {
      auto it=by_ns.find(anchor.ns);
      if(it==by_ns.end()) {l.reason="prior references absent state";return l;}
      const auto& f=*it->second;
      Eigen::Matrix<double,15,1> df;
      df.segment<3>(0)=imu::logSO3(anchor.q_WB.conjugate()*f.q_WB);
      df.segment<3>(3)=f.p_WB-anchor.p_WB;df.segment<3>(6)=f.v_W-anchor.v_W;
      df.segment<3>(9)=f.ba-anchor.ba;df.segment<3>(12)=f.bg-anchor.bg;
      Eigen::Matrix<double,15,15> derivative=Eigen::Matrix<double,15,15>::Identity();
      derivative.topLeftCorner<3,3>()=rightLogJacobian(df.head<3>());
      int first=anchor.ns==gauge_ns_?6:0;
      for(int k=first;k<15;++k) {
        delta[off+k-first]=df[k];
        for(int j=first;j<15;++j) D(off+k-first,column(l,f.ns,j))=derivative(k,j);
      }
      off+=15-first;
    }
    delta.segment<3>(off)=gravity_-prior_.gravity;
    D.block<3,3>(off,l.gravity_offset)=Eigen::Matrix3d::Identity();
    add(prior_.A*D,prior_.A*delta-prior_.rhs);
  }
  if(include_priors && initial_.active && (!removed || removed->count(gauge_ns_))) {
    auto it=by_ns.find(gauge_ns_);
    if(it==by_ns.end()) {l.reason="initial prior references absent state";return l;}
    const auto& f=*it->second;
    Mat J=Mat::Zero(9,dim);Vec r(9);
    r.segment<3>(0)=(f.v_W-initial_.v)/initial_.sigma_v;
    r.segment<3>(3)=(f.ba-initial_.ba)/initial_.sigma_ba;
    r.segment<3>(6)=(f.bg-initial_.bg)/initial_.sigma_bg;
    for(int k=0;k<9;++k) J(k,column(l,gauge_ns_,k+6))=
        1/(k<3?initial_.sigma_v:(k<6?initial_.sigma_ba:initial_.sigma_bg));
    add(J,r);
  }
  for(const auto& edge:edges_) {
    if(removed && !removed->count(edge.key.first)&&!removed->count(edge.key.second)) continue;
    const auto& fi=*by_ns.at(edge.key.first);const auto& fj=*by_ns.at(edge.key.second);
    imu::Preintegration pi;
    if(!imu::preintegrate(edge.raw,seconds(fi),seconds(fj),
                         fi.ba,fi.bg,options_.imu,&pi,&l.reason)) return l;
    auto si=toImu(fi),sj=toImu(fj);
    const Vec r=imu::whitenedResidual(pi,si,sj,gravity_);
    Mat J=Mat::Zero(15,dim);
    constexpr double eps=1e-6;
    for(int side=0;side<2;++side) {
      const auto& f=side==0?fi:fj;
      for(int k=0;k<15;++k) {
        const int c=column(l,f.ns,k);if(c<0) continue;
        Frame plus=f,minus=f;perturb(&plus,k,eps);perturb(&minus,k,-eps);
        const Vec rp=side==0?imu::whitenedResidual(pi,toImu(plus),sj,gravity_):
                             imu::whitenedResidual(pi,si,toImu(plus),gravity_);
        const Vec rm=side==0?imu::whitenedResidual(pi,toImu(minus),sj,gravity_):
                             imu::whitenedResidual(pi,si,toImu(minus),gravity_);
        J.col(c)=(rp-rm)/(2*eps);
      }
    }
    Eigen::Matrix<double,15,3> Jg=Eigen::Matrix<double,15,3>::Zero();
    Jg.block<3,3>(3,0)=-.5*pi.dt*pi.dt*fi.q_WB.toRotationMatrix().transpose();
    Jg.block<3,3>(6,0)=-pi.dt*fi.q_WB.toRotationMatrix().transpose();
    J.middleCols<3>(l.gravity_offset)=pi.sqrt_information*Jg;
    addSparse(J,r);
  }
  const auto mature=eligiblePoints();
  for(const auto& o:observations_) {
    if(!mature.count(o.point_id)) continue;
    if(absorbed && !absorbed->count(o.point_id)) continue;
    const auto& f=*by_ns.at(o.frame_ns);
    const Eigen::Matrix3d Rbw=f.q_WB.toRotationMatrix().transpose();
    const Eigen::Matrix3d Rcb=o.q_CB.toRotationMatrix();
    const Eigen::Vector3d pb=Rbw*(points_.at(o.point_id)-f.p_WB);
    const Eigen::Vector3d pc=Rcb*pb+o.t_CB;
    if(!pc.allFinite()||pc.z()<=1e-8) {l.reason="pixel projects behind camera";return l;}
    Eigen::Vector2d pixel=(pc.head<2>()/pc.z()-o.bearing.head<2>()/o.bearing.z()).cwiseProduct(o.focal);
    const double radius=pixel.norm(),h=options_.huber_pixels;
    if(!std::isfinite(h)||h<=0) {l.reason="invalid Huber radius";return l;}
    const double weight=radius>h?std::sqrt(h/radius):1;
    const double robust_cost=radius>h?h*radius-.5*h*h:.5*radius*radius;
    const Eigen::Vector2d r=weight*pixel/o.sigma_pixels;
    Eigen::Matrix<double,2,3> projection;
    projection << 1/pc.z(),0,-pc.x()/(pc.z()*pc.z()),
                  0,1/pc.z(),-pc.y()/(pc.z()*pc.z());
    projection=o.focal.asDiagonal()*projection*(weight/o.sigma_pixels);
    Mat J=Mat::Zero(2,dim);
    const Eigen::Matrix<double,2,3> Jrot=projection*Rcb*skew(pb);
    const Eigen::Matrix<double,2,3> Jpos=-projection*Rcb*Rbw;
    for(int k=0;k<6;++k) {
      const int c=column(l,o.frame_ns,k);
      if(c>=0) J.col(c)=k<3?Jrot.col(k):Jpos.col(k-3);
    }
    auto& block=l.points[o.point_id];
    if(!block.W.size()) block.W=Mat::Zero(dim,3);
    const Eigen::Matrix<double,2,3> Jp=projection*Rcb*Rbw;
    block.H.noalias()+=Jp.transpose()*Jp;block.b.noalias()-=Jp.transpose()*r;
    block.W.noalias()+=J.transpose()*Jp;
    addSparse(J,r);
    l.cost+=robust_cost/(o.sigma_pixels*o.sigma_pixels)-.5*r.squaredNorm();
  }
  for(auto& pair:l.points) {
    Mat inv;
    if(!psdInverse(pair.second.H,&inv,&pair.second.rank,&l.reason)) return l;
  }
  l.H=(l.H+l.H.transpose()).eval()*.5;
  l.finite=l.H.allFinite()&&l.b.allFinite()&&std::isfinite(l.cost);
  if(!l.finite) l.reason="nonfinite factor system";
  return l;
}
bool JointWindow::schur(const Linearization& l,Mat* H,Vec* b,std::string* error) {
  if(!l.finite) return fail(error,l.reason);
  *H=l.H;*b=l.b;
  for(const auto& pair:l.points) {
    const auto& p=pair.second;Mat inverse;
    if(!std::isfinite(p.numeric_scale)||p.numeric_scale<=0) return fail(error,"invalid point numerical scale");
    if(!psdInverse(p.H,&inverse,nullptr,error)) return false;
    inverse/=p.numeric_scale;
    H->noalias()-=p.W*inverse*p.W.transpose();b->noalias()-=p.W*inverse*p.b;
  }
  *H=(*H+H->transpose()).eval()*.5;
  return H->allFinite()&&b->allFinite();
}
std::map<int64_t,Eigen::Vector3d> JointWindow::backSubstitute(
    const Linearization& l,const Vec& dx) {
  std::map<int64_t,Eigen::Vector3d> result;
  for(const auto& pair:l.points) {
    const auto& p=pair.second;Mat inverse;
    if(!std::isfinite(p.numeric_scale)||p.numeric_scale<=0||!psdInverse(p.H,&inverse,nullptr)) return {};
    inverse/=p.numeric_scale;
    result[pair.first]=inverse*(p.b-p.W.transpose()*dx);
  }
  return result;
}
void JointWindow::applyStep(const Linearization& l,const Vec& dx,
    const std::map<int64_t,Eigen::Vector3d>& dp) {
  for(auto& f:frames_) {
    Eigen::Vector3d w=Eigen::Vector3d::Zero();
    for(int k=0;k<3;++k) {int c=column(l,f.ns,k);if(c>=0) w[k]=dx[c];}
    f.q_WB=(f.q_WB*imu::expSO3(w)).normalized();
    for(int k=3;k<15;++k) {int c=column(l,f.ns,k);if(c>=0) perturb(&f,k,dx[c]);}
  }
  gravity_+=dx.segment<3>(l.gravity_offset);
  for(const auto& p:dp) points_.at(p.first)+=p.second;
}
JointWindow::SolveReport JointWindow::status() const {
  auto r=report_;r.active_pixels=active_pixels_.size();r.absorbed_pixels=absorbed_pixels_.size();
  r.dropped_pixels=dropped_pixels_.size();
  const auto mature=eligiblePoints();
  r.pending_points=points_.size()-mature.size();r.pending_pixels=0;r.spatial_ineligible_points=0;
  for(const auto& p:points_)if(!p.second.allFinite()||p.second.norm()>kMaxPointNorm)++r.spatial_ineligible_points;
  for(const auto& o:observations_)if(!mature.count(o.point_id))++r.pending_pixels;
  r.active_imu_edges=active_edges_.size();r.absorbed_imu_edges=absorbed_edges_.size();
  r.active_points=points_.size();return r;
}
JointWindow::SolveReport JointWindow::optimize() {
  report_=SolveReport();
  if(options_.max_iterations<=0) {report_.reason="nonpositive iteration budget";return status();}
  auto l=linearize();
  if(!l.finite) {report_.reason=l.reason;return status();}
  report_.initial_cost=l.cost;report_.final_cost=l.cost;
  // Damping acts on state variables and the observable point subspace. The
  // same trial blocks drive elimination and recovery; none enters posterior.
  double lambda=1e-5;
  bool converged=false;
  for(int iter=0;iter<options_.max_iterations;++iter) {
    Mat H;Vec b;
    if(!schur(l,&H,&b,&report_.reason)) return status();
    Mat inverse;int rank=0;
    if(!psdInverse(H,&inverse,&rank,&report_.reason)) return status();
    report_.state_dimension=H.rows();report_.state_rank=rank;
    report_.iterations=iter+1;
    double gradient=l.b.norm();
    for(const auto& point:l.points) gradient=std::max(gradient,point.second.b.norm());
    if(gradient<=1e-10*(1+std::sqrt(std::max(0.0,l.cost)))) {converged=true;break;}
    const auto saved_frames=frames_;const auto saved_points=points_;const auto saved_g=gravity_;
    bool accepted=false;
    for(int trial=0;trial<12;++trial) {
      Linearization trial_system=l;
      for(int k=0;k<l.H.rows();++k)
        trial_system.H(k,k)+=lambda*std::max(1.0,std::abs(l.H(k,k)));
      for(auto& point:trial_system.points) point.second.numeric_scale=1+lambda;
      Mat damped;Vec trial_b;
      if(!schur(trial_system,&damped,&trial_b)) {lambda*=10;continue;}
      Eigen::LDLT<Mat> ldlt(damped);
      if(ldlt.info()!=Eigen::Success || !ldlt.isPositive()) {lambda*=10;continue;}
      const Vec dx=ldlt.solve(trial_b);
      if(!dx.allFinite()) {lambda*=10;continue;}
      const auto dp=backSubstitute(trial_system,dx);
      frames_=saved_frames;points_=saved_points;gravity_=saved_g;
      applyStep(l,dx,dp);
      // Admission cannot be evaded by moving a participating free point out
      // of MAX_DIST and making its likelihood disappear from the next cost.
      bool feasible=true;
      for(const auto& point:l.points) {
        const auto& position=points_.at(point.first);
        if(!position.allFinite()||position.norm()>kMaxPointNorm) {feasible=false;break;}
      }
      if(!feasible) {lambda*=10;continue;}
      auto next=linearize();
      if(next.finite && next.cost<=l.cost) {
        report_.max_step=dx.cwiseAbs().maxCoeff();
        for(const auto& p:dp) report_.max_step=std::max(report_.max_step,p.second.cwiseAbs().maxCoeff());
        const double decrease=l.cost-next.cost;
        converged=report_.max_step<1e-7 || decrease<1e-10*(1+l.cost);
        l=std::move(next);report_.final_cost=l.cost;
        lambda=std::max(1e-12,lambda*.2);accepted=true;break;
      }
      lambda*=10;
    }
    if(!accepted) {
      frames_=saved_frames;points_=saved_points;gravity_=saved_g;
      report_.reason="no cost-decreasing LM step";break;
    }
    if(converged) break;
  }
  Mat H;Vec b;Mat inverse;int rank=0;
  if(!schur(l,&H,&b,&report_.reason)||!psdInverse(H,&inverse,&rank,&report_.reason)) return status();
  report_.state_dimension=H.rows();report_.state_rank=rank;
  report_.finite=l.finite;report_.final_cost=l.cost;
  report_.success=converged;
  if(converged) report_.reason="numerical convergence; physical acceptance is external";
  else if(report_.reason.empty()) report_.reason="iteration budget exhausted";
  return status();
}
Mat JointWindow::covariance(int* rank) const {
  auto l=linearize();Mat H,inverse;Vec b;
  if(!schur(l,&H,&b)||!psdInverse(H,&inverse,rank)) {if(rank)*rank=0;return Mat();}
  return inverse;
}

bool JointWindow::marginalizeTo(size_t keep,std::string* error) {
  if(keep==0) return fail(error,"cannot marginalize entire window");
  if(frames_.size()<=keep) return true;
  std::set<int64_t> removed,absorbed;
  const size_t count=frames_.size()-keep;
  for(size_t k=0;k<count;++k) removed.insert(frames_[k].ns);
  for(const auto& o:observations_) if(removed.count(o.frame_ns)) absorbed.insert(o.point_id);
  const auto mature=eligiblePoints();
  auto l=assemble(&removed,&absorbed,true);
  Mat H;Vec b;
  if(!schur(l,&H,&b,error)) return false;
  std::vector<int> eliminate,retain;
  for(const auto& f:frames_) for(int k=0;k<15;++k) {
    int c=column(l,f.ns,k);if(c<0)continue;
    (removed.count(f.ns)?eliminate:retain).push_back(c);
  }
  for(int k=0;k<3;++k) retain.push_back(l.gravity_offset+k);
  auto submat=[&](const std::vector<int>& rows,const std::vector<int>& cols) {
    Mat result(rows.size(),cols.size());
    for(size_t i=0;i<rows.size();++i)for(size_t j=0;j<cols.size();++j)result(i,j)=H(rows[i],cols[j]);
    return result;
  };
  Mat Hmm=submat(eliminate,eliminate),Hkm=submat(retain,eliminate),Hkk=submat(retain,retain);
  Vec bm(eliminate.size()),bk(retain.size());
  for(size_t k=0;k<eliminate.size();++k)bm[k]=b[eliminate[k]];
  for(size_t k=0;k<retain.size();++k)bk[k]=b[retain[k]];
  Mat inverse;
  if(!psdInverse(Hmm,&inverse,nullptr,error)) return false;
  Hkk.noalias()-=Hkm*inverse*Hkm.transpose();bk.noalias()-=Hkm*inverse*bm;
  Hkk=(Hkk+Hkk.transpose()).eval()*.5;
  Eigen::SelfAdjointEigenSolver<Mat> eig(Hkk);
  if(eig.info()!=Eigen::Success) return fail(error,"marginal prior eigensolve failed");
  const double scale=std::max(1.0,eig.eigenvalues().cwiseAbs().maxCoeff());
  const double tolerance=128*std::numeric_limits<double>::epsilon()*Hkk.rows()*scale;
  if(eig.eigenvalues().minCoeff() < -32*tolerance)
    return fail(error,"marginal prior is indefinite");
  int rank=0;for(int k=0;k<Hkk.rows();++k)if(eig.eigenvalues()[k]>tolerance)++rank;
  Prior next;
  next.A=Mat::Zero(rank,Hkk.cols());next.rhs=Vec::Zero(rank);
  int row=0;
  for(int k=0;k<Hkk.rows();++k) if(eig.eigenvalues()[k]>tolerance) {
    const double root=std::sqrt(eig.eigenvalues()[k]);
    next.A.row(row)=root*eig.eigenvectors().col(k).transpose();
    next.rhs[row]=eig.eigenvectors().col(k).dot(bk)/root;++row;
  }
  next.anchors.assign(frames_.begin()+count,frames_.end());next.gravity=gravity_;
  // Only after successful factorization mutate ownership. Entire point segments
  // are retired, including every retained-frame row. Immature rows carried
  // no likelihood information and are dropped rather than labeled absorbed.
  prior_=std::move(next);
  const int64_t retirement_ns=frames_.back().ns;
  for(int64_t id:absorbed) {points_.erase(id);retired_after_[id]=retirement_ns;}
  observations_.erase(std::remove_if(observations_.begin(),observations_.end(),[&](const Observation& o){
    if(!absorbed.count(o.point_id))return false;
    PixelKey key(o.frame_ns,o.camera_id,o.feature_id);
    active_pixels_.erase(key);
    if(mature.count(o.point_id))absorbed_pixels_.insert(key);else dropped_pixels_.insert(key);
    return true;
  }),observations_.end());
  edges_.erase(std::remove_if(edges_.begin(),edges_.end(),[&](const Edge& edge){
    if(!removed.count(edge.key.first)&&!removed.count(edge.key.second))return false;
    active_edges_.erase(edge.key);absorbed_edges_.insert(edge.key);return true;
  }),edges_.end());
  if(removed.count(gauge_ns_))initial_.active=false;
  frames_.erase(frames_.begin(),frames_.begin()+count);
  report_=SolveReport();return true;
}
}  // namespace schur_vins
