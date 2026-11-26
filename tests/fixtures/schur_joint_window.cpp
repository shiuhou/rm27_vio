#include "joint_window.h"
#include <Eigen/QR>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <stdexcept>

using Window=schur_vins::JointWindow;
using Mat=Eigen::MatrixXd;
using Vec=Eigen::VectorXd;
namespace imu=svo::headless::joint_imu;
namespace {
void check(bool condition,const std::string& why) {
  if(!condition) throw std::runtime_error(why);
}
Window::Options options() {
  Window::Options o;
  o.imu.sigma_acc=.008;o.imu.sigma_gyro=.0012;
  o.imu.sigma_ba=.1;o.imu.sigma_bg=.03;o.max_iterations=25;
  return o;
}
Window::Frame frame(int k) {
  Window::Frame f;f.ns=1000000000LL+50000000LL*k;
  double t=.05*k;f.timestamp=1+t;
  f.q_WB=imu::expSO3(Eigen::Vector3d(0,0,.15*t));
  f.p_WB=Eigen::Vector3d(.4*t+.2*t*t,.05*t*t,.03*t*t);
  f.v_W=Eigen::Vector3d(.4+.4*t,.1*t,.06*t);
  return f;
}
Window::Measurements measurements(int k) {
  Window::Measurements m;
  for(int j=0;j<=10;++j) {
    double t=.05*(k-1)+.005*j;
    imu::Sample s;s.timestamp=1+t;
    s.angular_velocity=Eigen::Vector3d(0,0,.15);
    Eigen::Quaterniond q=imu::expSO3(Eigen::Vector3d(0,0,.15*t));
    s.linear_acceleration=q.conjugate()*Eigen::Vector3d(.4,.1,9.87007);
    m.push_back(s);
  }
  m.front().timestamp=frame(k-1).timestamp;
  m.back().timestamp=frame(k).timestamp;
  return m;
}
Window::Observation observation(int k,int64_t point,const Eigen::Vector3d& p) {
  auto f=frame(k);Window::Observation o;
  o.frame_ns=f.ns;o.point_id=point;o.feature_id=point+1000;
  o.bearing=(f.q_WB.conjugate()*(p-f.p_WB)).normalized();
  o.focal=Eigen::Vector2d(400,403);o.sigma_pixels=1;
  return o;
}
void populate(Window* w) {
  std::string err;
  for(int k=0;k<4;++k) {
    auto f=frame(k);
    if(k) {f.p_WB+=Eigen::Vector3d(.0004*k,-.0003*k,.0001*k);
           f.q_WB=(f.q_WB*imu::expSO3(Eigen::Vector3d(.0001*k,0,-.0002*k))).normalized();}
    check(w->addFrame(f,k?measurements(k):Window::Measurements(),&err),err);
  }
  check(w->setGravity(Eigen::Vector3d(.001,-.002,-9.805),&err),err);
  check(w->setInitialPrior(frame(0).v_W,Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),2,.1,.01,&err),err);
  for(int i=0;i<10;++i) {
    const Eigen::Vector3d p((i%5-2)*.4,(i/5-.5)*.5,3.5+.1*(i%3));
    check(w->addLandmark(i,p+Eigen::Vector3d(.002,-.001,.01),&err),err);
    // Points 8/9 exist only in retained frames and must stay raw.
    for(int k=i<8?0:2;k<4;++k)check(w->addObservation(observation(k,i,p),&err),err);
  }
}
void denseEquivalence() {
  // A known dense Gaussian with nonzero residual and cross-correlations. Dense
  // QR is independent of the production point elimination/recovery path.
  const int nx=6,np=2,rows=26,n=nx+3*np;
  Mat J(rows,n);
  for(int i=0;i<rows;++i)for(int j=0;j<n;++j)
    J(i,j)=std::sin(.37*(i+1)*(j+2))+.13*std::cos(.21*(i+2)*(j+1));
  // One factor may depend on one point only; enforce exact point block structure.
  J.topRows(13).rightCols(3).setZero();
  J.bottomRows(13).middleCols(nx,3).setZero();
  Vec r(rows);for(int i=0;i<rows;++i)r[i]=std::cos(.17*(i+2));
  Window::Linearization l;l.finite=true;
  const Mat H=J.transpose()*J;const Vec b=-J.transpose()*r;
  l.H=H.topLeftCorner(nx,nx);l.b=b.head(nx);
  for(int p=0;p<np;++p) {
    Window::PointBlock block;
    block.H=H.block<3,3>(nx+3*p,nx+3*p);
    block.b=b.segment<3>(nx+3*p);block.W=H.block(0,nx+3*p,nx,3);
    l.points[p]=block;
  }
  Mat S;Vec c;check(Window::schur(l,&S,&c),"synthetic Schur");
  Vec dx=S.ldlt().solve(c);auto dp=Window::backSubstitute(l,dx);
  Vec actual(n);actual.head(nx)=dx;
  for(int p=0;p<np;++p)actual.segment<3>(nx+3*p)=dp.at(p);
  const Vec expected=J.colPivHouseholderQr().solve(-r);
  check((actual-expected).norm()<1e-10,"dense QR vs Schur/backsub mismatch");
  check((H*actual-b).norm()<1e-10,"full joint normal equation mismatch");
  // One-view point has a true zero-information depth direction.
  Window::Linearization singular;
  singular.finite=true;singular.H=Mat::Identity(2,2);singular.b=Vec::Zero(2);
  Window::PointBlock one;one.W=Mat::Zero(2,3);one.H.diagonal()<<4,9,0;
  one.b=Eigen::Vector3d(2,-3,0);singular.points[8]=one;
  auto point_step=Window::backSubstitute(singular,Vec::Zero(2));
  check((point_step.at(8)-Eigen::Vector3d(.5,-1./3,0)).norm()<1e-12,"nullspace acquired point information");
}
void dampedObservableSubspace() {
  // Exact C4 point436 blocks (17-digit dump bound to c4 tracking-iterate).
  // Its 9.43e-15 weak eigenvalue was incorrectly promoted by H*(1+lambda).
  Window::PointBlock p;p.rank=2;
  p.H<<.0075766786470100852,.0050858908905166847,-.0019358903349435883,
       .0050858908905166847,.0048518393753133899,.0015440524559360647,
       -.0019358903349435883,.0015440524559360647,.0061178601239817096;
  p.b<<.00018215557250764181,.00011281694385836816,-.000065247275729114068;
  p.W.resize(12,3);
  p.W<<25.311014926818228,17.655224305819612,-5.1519370626706413,
    -.41302817772754341,-7.1967858427017921,-13.578249587246884,
    1.3859272737529649,-4.2471004765970859,-10.592740845800391,
    -.0046193442339359375,-.0030969570929447747,.0011877960776813517,
    -.0030969570929447747,-.0029537922907742972,-.00093895475258542072,
    .0011877960776813517,-.00093895475258542072,-.0037370619214300432,
    16.205551405363018,11.377496561961392,-3.153000559229739,
    -.23461922567667454,-4.5666557639782663,-8.6593679383301918,
    .93987011607647708,-2.6819726035971305,-6.7914776826894219,
    -.0029573344130741476,-.00198893379757191,.00074809425726223662,
    -.00198893379757191,-.0018980470845390922,-.00060509770335064397,
    .00074809425726223662,-.00060509770335064397,-.0023807982025516664;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(p.H);
  const double cutoff=128*std::numeric_limits<double>::epsilon()*3;
  check(eig.eigenvalues()[0]<cutoff && eig.eigenvalues()[0]*11>cutoff,
        "C4 witness does not expose old scale-dependent rank flip");
  const Eigen::Matrix<double,3,2> U=eig.eigenvectors().rightCols<2>();
  const Eigen::Matrix2d D=eig.eigenvalues().tail<2>().asDiagonal();
  const Mat pinv=U*D.inverse()*U.transpose();
  Window::Linearization base;base.finite=true;base.points[436]=p;
  base.H=p.W*pinv*p.W.transpose()+1e6*Mat::Identity(12,12);
  base.b=p.W*pinv*p.b+Vec::Constant(12,.01);
  for(double lambda:{0.,1e-5,.001,.1,1.,10.,100.,1e4,1e8}) {
    auto trial=base;trial.points.at(436).numeric_scale=1+lambda;
    for(int k=0;k<12;++k)trial.H(k,k)+=lambda*std::max(1.,std::abs(base.H(k,k)));
    Mat S;Vec b;check(Window::schur(trial,&S,&b),"damped C4 Schur");
    const Vec dx=S.ldlt().solve(b);const auto dp=Window::backSubstitute(trial,dx).at(436);
    // Independent dense Gaussian in the TWO frozen observable coordinates.
    Mat dense=Mat::Zero(14,14);dense.topLeftCorner(12,12)=trial.H;
    dense.topRightCorner(12,2)=p.W*U;dense.bottomLeftCorner(2,12)=U.transpose()*p.W.transpose();
    dense.bottomRightCorner<2,2>()=(1+lambda)*D;
    Vec rhs(14);rhs.head(12)=base.b;rhs.tail<2>()=U.transpose()*p.b;
    const Vec expected=dense.ldlt().solve(rhs);
    check((dx-expected.head(12)).norm()<1e-11 && (dp-U*expected.tail<2>()).norm()<1e-10,
          "damped Schur/backsub differs from dense frozen-subspace model");
    check(std::abs(eig.eigenvectors().col(0).dot(dp))<1e-11 && dp.norm()<1 &&
          trial.points.at(436).rank==2 && (trial.points.at(436).H-p.H).norm()==0,
          "damping activated a null mode or kilometer point step");
  }
  // This degeneracy can occur after the original THREE-view maturity gate,
  // so admission alone does not replace the rank-preserving damping fix.
  Window w(options());std::string error;const Eigen::Vector3d far(100,50,450);
  for(int k=0;k<3;++k) {
    auto f=frame(k);f.q_WB=Eigen::Quaterniond::Identity();f.p_WB=Eigen::Vector3d(k*.0005,0,0);
    check(w.addFrame(f,k?measurements(k):Window::Measurements(),&error),error);
  }
  check(w.addLandmark(44,far,&error),error);
  for(int k=0;k<3;++k) {
    auto o=observation(k,44,far);o.bearing=(far-w.frames()[k].p_WB).normalized();o.focal.setConstant(45);
    check(w.addObservation(o,&error),error);
  }
  const auto raw=w.linearize();check(raw.finite && raw.points.at(44).rank==2 &&
      w.status().pending_pixels==0,"three-view near-rank-two point admission regression");
}
void marginalEquivalence() {
  Window w(options());populate(&w);
  auto full=w.linearize();Mat H;Vec b;
  check(Window::schur(full,&H,&b),"pre-marginal Schur");
  // First fixed pose omits exactly six columns; only its v,ba,bg are removed.
  check(full.frame_offsets.at(frame(0).ns)==0 && full.gravity_offset==54,"gauge layout");
  const int old=9,n=H.rows()-old;
  // Independent dense elimination of old state AND all points in one block,
  // using LDLT. This does not repeat the engine's per-point Schur/eigen path.
  const int np=full.points.size(), total=H.rows()+3*np;
  Mat joint=Mat::Zero(total,total);Vec jointb=Vec::Zero(total);
  joint.topLeftCorner(H.rows(),H.rows())=full.H;jointb.head(H.rows())=full.b;
  int poff=H.rows();
  for(const auto& pair:full.points) {
    const auto& p=pair.second;
    joint.block(poff,poff,3,3)=p.H;jointb.segment<3>(poff)=p.b;
    joint.block(0,poff,H.rows(),3)=p.W;
    joint.block(poff,0,3,H.rows())=p.W.transpose();poff+=3;
  }
  std::vector<int> eliminated;
  for(int c=0;c<old;++c)eliminated.push_back(c);
  for(int c=H.rows();c<total;++c)eliminated.push_back(c);
  Mat marginal(eliminated.size(),eliminated.size()),cross(n,eliminated.size());
  Vec bm(eliminated.size());
  for(size_t i=0;i<eliminated.size();++i) {
    bm[i]=jointb[eliminated[i]];
    cross.col(i)=joint.block(old,eliminated[i],n,1);
    for(size_t j=0;j<eliminated.size();++j)marginal(i,j)=joint(eliminated[i],eliminated[j]);
  }
  Eigen::LDLT<Mat> dense_elimination(marginal);
  check(dense_elimination.info()==Eigen::Success && dense_elimination.isPositive(),"dense marginal oracle not SPD");
  const Mat expectedH=joint.block(old,old,n,n)-cross*dense_elimination.solve(cross.transpose());
  const Vec expectedb=jointb.segment(old,n)-cross*dense_elimination.solve(bm);
  std::string error;
  check(w.marginalizeTo(3,&error),error);
  auto retained=w.linearize();Mat actualH;Vec actualb;
  check(Window::schur(retained,&actualH,&actualb),"post-marginal Schur");
  const double dh=(actualH-expectedH).norm()/expectedH.norm();
  const double db=(actualb-expectedb).norm()/expectedb.norm();
  check(dh<2e-9,"marginal information differs from dense elimination "+std::to_string(dh));
  check(db<2e-7,"marginal gradient differs from dense elimination "+std::to_string(db));
  auto status=w.status();
  check(status.absorbed_pixels==32 && status.active_pixels==4 &&
        status.absorbed_imu_edges==1 && status.active_imu_edges==2,"factor closure/counts");
  check(w.landmarks().size()==2,"entire incident point segments must retire");
  // Frozen manifold prior remains anchored; check its right-SO(3) derivative
  // after moving the retained mean away from that anchor.
  const auto original_retained=w.frames().front();
  auto moved=original_retained;
  moved.q_WB=(moved.q_WB*imu::expSO3(Eigen::Vector3d(.01,-.02,.015))).normalized();
  moved.p_WB+=Eigen::Vector3d(.001,-.002,.001);
  check(w.setFrameSeed(moved,&error),error);
  auto at_moved=w.linearize();
  constexpr double eps=1e-6;
  for(int axis=0;axis<3;++axis) {
    Eigen::Vector3d tangent=Eigen::Vector3d::Zero();tangent[axis]=eps;
    auto plus=moved,minus=moved;
    plus.q_WB=(moved.q_WB*imu::expSO3(tangent)).normalized();
    minus.q_WB=(moved.q_WB*imu::expSO3(-tangent)).normalized();
    check(w.setFrameSeed(plus,&error),error);const double cp=w.linearize().cost;
    check(w.setFrameSeed(minus,&error),error);const double cm=w.linearize().cost;
    const double numerical=(cp-cm)/(2*eps);
    check(std::abs(at_moved.b[axis]+numerical)<1e-5*(1+std::abs(numerical)),"manifold marginal prior Jacobian");
  }
  check(w.setFrameSeed(original_retained,&error),error);
  check(w.addLandmark(0,Eigen::Vector3d(0,0,4),&error),error);
  check(!w.addObservation(observation(1,0,Eigen::Vector3d(0,0,4)),&error),"absorbed retained pixel reused");
  auto different_pixel=observation(2,0,Eigen::Vector3d(0,0,4));different_pixel.feature_id+=100000;
  check(!w.addObservation(different_pixel,&error),"new segment reopened historical measurement");
  check(w.addFrame(frame(4),measurements(4),&error),error);
  check(w.addObservation(observation(4,0,Eigen::Vector3d(0,0,4)),&error),error);
  check(!w.addObservation(observation(4,0,Eigen::Vector3d(0,0,4)),&error),"active pixel duplicated");
  auto reassigned=observation(4,0,Eigen::Vector3d(0,0,4));reassigned.point_id=8;
  check(!w.addObservation(reassigned,&error),"same pixel reassigned to another segment");
  check(!w.addFrame(frame(4),measurements(4),&error),"IMU edge replayed");
  // Another marginalization includes the complete existing state-only prior.
  check(w.marginalizeTo(2,&error),error);
  check(w.linearize().finite,"repeated marginal prior");
  std::cout<<"marginal_H_relative_error="<<dh<<" marginal_b_relative_error="<<db<<" ";
}
void currentRowsAndRejection() {
  Window w(options());populate(&w);std::string error;
  check(w.currentMultiViewObservations(frame(3).ns)==8,"two-view tracks passed MIN_OBS=3");
  for(int i=8;i<10;++i) {
    const Eigen::Vector3d p((i%5-2)*.4,(i/5-.5)*.5,3.5+.1*(i%3));
    check(w.addObservation(observation(1,i,p),&error),error);
  }
  check(w.currentMultiViewObservations(frame(3).ns)==10,"three-view tracks failed maturity gate");
  check(w.currentMultiViewObservations(frame(0).ns)==0,"fixed gauge counted as camera correction");
  check(w.currentMultiViewObservations(frame(8).ns)==0,"unknown current frame counted");
  const Eigen::Vector3d p(.1,.2,4);
  check(w.addLandmark(200,p,&error),error);
  auto one=observation(3,200,p);
  check(w.addObservation(one,&error),error);
  auto stereo=one;stereo.camera_id=1;stereo.feature_id+=1;stereo.t_CB.x()=-.1;
  stereo.bearing=(frame(3).q_WB.conjugate()*(p-frame(3).p_WB)+stereo.t_CB).normalized();
  check(w.addObservation(stereo,&error),error);
  check(w.currentMultiViewObservations(frame(3).ns)==10,"same-frame stereo is not a distinct frame view");
  check(w.rejectObservation(frame(3).ns,0,1000,&error),error);
  check(w.currentMultiViewObservations(frame(3).ns)==9,"rejected current row still counted");
  check(!w.rejectObservation(frame(3).ns,0,1000,&error),"same rejection succeeded twice");
  auto reassigned=observation(3,200,p);reassigned.feature_id=1000;
  check(!w.addObservation(reassigned,&error),"dropped pixel reused under another point ID");
  check(w.rejectObservation(frame(1).ns,0,1009,&error),error);
  check(w.rejectObservation(frame(2).ns,0,1009,&error),error);
  check(w.rejectObservation(frame(3).ns,0,1009,&error),error);
  check(!w.landmarks().count(9),"unobserved free point was retained indefinitely");
  check(w.currentMultiViewObservations(frame(3).ns)==8,"dropped track still counted");
  auto status=w.status();
  check(status.dropped_pixels==4 && status.absorbed_pixels==0 && status.active_pixels==36,
        "dropped information mislabeled as absorbed");
  check(w.marginalizeTo(2,&error),error);
  check(w.status().dropped_pixels==4 && w.status().absorbed_pixels==34,
        "marginalization reabsorbed dropped pixels");
  check(!w.addObservation(reassigned,&error),"marginalization forgot dropped identity");
  // Different frame identities with identical optical centers still have no
  // depth information; repeated rays must not qualify a correction.
  Window degenerate(options());
  check(degenerate.addFrame(frame(0),{},&error),error);
  auto stationary=frame(0);stationary.ns=frame(1).ns;stationary.timestamp=frame(1).timestamp;
  check(degenerate.addFrame(stationary,measurements(1),&error),error);
  check(degenerate.addLandmark(1,p,&error),error);
  auto a=observation(0,1,p),b=a;b.frame_ns=stationary.ns;
  check(degenerate.addObservation(a,&error),error);check(degenerate.addObservation(b,&error),error);
  auto third=stationary;third.ns=frame(2).ns;third.timestamp=frame(2).timestamp;
  check(degenerate.addFrame(third,measurements(2),&error),error);
  auto repeated=b;repeated.frame_ns=third.ns;
  check(degenerate.addObservation(repeated,&error),error);
  check(degenerate.currentMultiViewObservations(third.ns)==0,"rank-two three-view repeated ray qualified");
  // Rank-three point geometry from current stereo alone cannot qualify temporal
  // correction when the old-frame row carries effectively zero information.
  a.sigma_pixels=1e20;
  Window no_temporal(options());
  check(no_temporal.addFrame(frame(0),{},&error),error);
  check(no_temporal.addFrame(stationary,measurements(1),&error),error);
  check(no_temporal.addLandmark(1,p,&error),error);
  check(no_temporal.addObservation(a,&error),error);check(no_temporal.addObservation(b,&error),error);
  auto c=b;c.camera_id=1;c.t_CB.x()=-.1;
  c.bearing=(p+c.t_CB).normalized();
  check(no_temporal.addObservation(c,&error),error);
  check(no_temporal.addFrame(third,measurements(2),&error),error);
  auto weak=a;weak.frame_ns=third.ns;
  check(no_temporal.addObservation(weak,&error),error);
  check(no_temporal.currentMultiViewObservations(stationary.ns)==0,
        "zero reduced temporal camera information qualified with three frame views");
}
void pendingRetirement() {
  Window w(options());std::string error;
  for(int k=0;k<3;++k)check(w.addFrame(frame(k),k?measurements(k):Window::Measurements(),&error),error);
  const Eigen::Vector3d p(0,0,4);
  check(w.addLandmark(20,p,&error),error);
  check(w.addObservation(observation(0,20,p),&error),error);
  check(w.addObservation(observation(2,20,p),&error),error);
  const auto before=w.status();
  check(before.active_pixels==2 && before.pending_pixels==2 && w.linearize().points.empty(),
        "immature point was linearized");
  check(w.marginalizeTo(2,&error),error);
  const auto after=w.status();
  check(after.absorbed_pixels==0 && after.dropped_pixels==2 && after.active_pixels==0 &&
        after.pending_pixels==0 && after.pending_points==0 && !w.landmarks().count(20),
        "pending closure labeled as absorbed information");
  check(w.addLandmark(20,p,&error),error);
  auto old=observation(2,20,p);old.feature_id+=10000;
  check(!w.addObservation(old,&error),"pending retired physical segment reopened historical row");
}
void spatialAdmission() {
  Window::Options o=options();o.max_iterations=1;
  Window w(o);std::string error;
  for(int k=0;k<3;++k) {
    auto f=frame(k);f.q_WB=Eigen::Quaterniond::Identity();f.p_WB=Eigen::Vector3d(k,0,0);f.v_W=Eigen::Vector3d(20,0,0);
    auto raw=k?measurements(k):Window::Measurements();
    for(auto& sample:raw) {sample.angular_velocity.setZero();sample.linear_acceleration=Eigen::Vector3d(0,0,9.81007);}
    check(w.addFrame(f,raw,&error),error);
  }
  check(w.setInitialPrior(Eigen::Vector3d(20,0,0),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),2,.1,.01,&error),error);
  check(w.addLandmark(50,Eigen::Vector3d(0,0,999),&error),error);
  check(w.addLandmark(51,Eigen::Vector3d(0,0,2000),&error),error);
  for(int k=0;k<3;++k)for(int id:{50,51}) {
    auto row=observation(k,id,Eigen::Vector3d(0,0,2000));
    row.bearing=(Eigen::Vector3d(0,0,2000)-w.frames()[k].p_WB).normalized();
    check(w.addObservation(row,&error),error);
  }
  auto l=w.linearize();
  check(l.points.count(50)==1 && l.points.count(51)==0 && w.admittedPoints()==std::set<int64_t>{50} &&
        w.status().pending_pixels==3 &&
        w.status().spatial_ineligible_points==1 && w.currentMultiViewObservations(frame(2).ns)==1,
        "original MAX_DIST admission gate relaxed");
  auto trial=l;const double lambda=1e-5;
  for(int k=0;k<trial.H.rows();++k)trial.H(k,k)+=lambda*std::max(1.,std::abs(l.H(k,k)));
  for(auto& p:trial.points)p.second.numeric_scale=1+lambda;
  Mat S;Vec b;check(Window::schur(trial,&S,&b),"spatial trial Schur");
  const Vec dx=S.ldlt().solve(b);const auto dp=Window::backSubstitute(trial,dx);
  check((w.landmarks().at(50)+dp.at(50)).norm()>1000,"fixture did not demand out-of-bound point trial");
  w.optimize();
  check(w.landmarks().at(50).norm()<=1000 && w.linearize().points.count(50)==1,
        "LM escaped MAX_DIST to discard a participating likelihood");
  check(w.marginalizeTo(2,&error),error);
  check(w.status().absorbed_pixels==3 && w.status().dropped_pixels==3,
        "spatial-ineligible pending rows mislabeled as absorbed");
}
void nonlinearAndConsumption() {
  Window w(options());populate(&w);
  const auto gauge=w.frames().front();const auto before=w.status();
  const double initial=w.linearize().cost;
  const auto report=w.optimize();
  check(report.finite && report.final_cost<initial,"nonlinear common objective did not decrease");
  check((w.frames().front().p_WB-gauge.p_WB).norm()==0 &&
        w.frames().front().q_WB.coeffs()==gauge.q_WB.coeffs(),"gauge changed");
  const auto after=w.status();
  check(before.active_pixels==after.active_pixels && before.active_imu_edges==after.active_imu_edges &&
        after.absorbed_pixels==0,"relinearization changed measurement ownership");
  int rank=0;auto covariance=w.covariance(&rank);
  check(covariance.allFinite() && covariance.rows()==57 && rank>0,"undamped covariance/rank");
  const auto all=w.linearize(),data=w.linearize(false);
  check((all.H-data.H).norm()>1,"initial prior missing or data-only includes it");
  // One/two-view segments are pending, including their cost and unknowns.
  // Three repeated optical centers mature the segment without adding depth;
  // a pure point bearing error must then still run point recovery.
  Window point_only(options());std::string error;
  auto fixed=frame(0);fixed.v_W.setZero();
  check(point_only.addFrame(fixed,{},&error),error);
  check(point_only.setInitialPrior(Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),
      Eigen::Vector3d::Zero(),2,.1,.01,&error),error);
  const Eigen::Vector3d truth(.3,.2,4),seed(.6,.1,4);
  check(point_only.addLandmark(7,seed,&error),error);
  check(point_only.addObservation(observation(0,7,truth),&error),error);
  check(point_only.linearize().cost==0 && point_only.linearize().points.empty() &&
        point_only.status().pending_pixels==1 && point_only.status().pending_points==1,
        "one-view raw factor entered objective");
  for(int k=1;k<=2;++k) {
    auto stationary=fixed;stationary.ns=frame(k).ns;stationary.timestamp=frame(k).timestamp;
    auto raw=measurements(k);
    for(auto& sample:raw) {sample.angular_velocity.setZero();sample.linear_acceleration=Eigen::Vector3d(0,0,9.81007);}
    check(point_only.addFrame(stationary,raw,&error),error);
    auto row=observation(0,7,truth);row.frame_ns=stationary.ns;
    check(point_only.addObservation(row,&error),error);
    if(k==1)check(point_only.linearize().points.empty() && point_only.status().pending_pixels==2,
                  "two-view raw factor entered objective");
  }
  check(point_only.status().pending_pixels==0 && point_only.linearize().points.at(7).rank==2,
        "three-view rank-two point did not enter observable subspace");
  const double point_cost=point_only.linearize().cost;
  auto result=point_only.optimize();
  check(result.finite && result.final_cost<point_cost*1e-6 &&
        (point_only.landmarks().at(7)-seed).norm()>.01,"pure point step skipped");
  std::cout<<"nonlinear_initial_cost="<<initial<<" final_cost="<<report.final_cost<<" rank="<<rank<<" ";
}
}
int main() {
  try {denseEquivalence();dampedObservableSubspace();marginalEquivalence();currentRowsAndRejection();pendingRetirement();spatialAdmission();nonlinearAndConsumption();
    std::cout<<"PASS\n";return 0;
  } catch(const std::exception& e) {std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
