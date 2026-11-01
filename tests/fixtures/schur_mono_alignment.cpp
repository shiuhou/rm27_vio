// Exercise the production bootstrap against known motion and true degeneracies.
// Synthetic motion only: this is not dataset or hardware qualification.
#include <svo/rm27_mono_alignment.h>
#include <iostream>
#include <string>
using namespace rm27;
const V3 g(0,0,-9.81007), ba(.03,-.02,.01), bg(.003,-.002,.001), lever(.04,-.02,.01);
Eigen::Quaterniond orientation(double t,bool rotation) {
 if(!rotation)return Eigen::Quaterniond::Identity();
 return expq(V3(.5*std::sin(1.9*t),.4*std::sin(1.3*t),.6*std::sin(.9*t)));
}
MonoAlignment make(bool moving,bool rotating,bool exciting) {
 MonoAlignment a; const double dt=.005,scale=2.3;
 V3 pos=V3::Zero(),vel=moving?V3(.3,-.1,.05):V3::Zero();
 std::vector<svo::ImuMeasurement,Eigen::aligned_allocator<svo::ImuMeasurement>> samples;
 std::vector<V3,Eigen::aligned_allocator<V3>> positions;
 for(int k=0;k<=600;++k){const double t=k*dt;auto q=orientation(t,rotating);
  V3 acceleration=V3::Zero();if(moving && exciting)acceleration=V3(1.5*std::sin(2.3*t),1.2*std::cos(1.7*t),.8*std::sin(3.1*t));
  const V3 gyro=logq(q.inverse()*orientation(t+1e-5,rotating))/1e-5;
  samples.emplace_back(t,gyro+bg,q.inverse()*(acceleration-g)+ba);
  positions.push_back(pos);
  const double tm=t+dt/2;V3 ac=V3::Zero();if(moving && exciting)ac=V3(1.5*std::sin(2.3*tm),1.2*std::cos(1.7*tm),.8*std::sin(3.1*tm));
  pos+=vel*dt+.5*ac*dt*dt;vel+=ac*dt;
 }
 for(int k=0;k<=600;k+=10){MonoPose p;p.t=k*dt;p.q=orientation(p.t,rotating);p.camera=(positions[k]+p.q*lever)/scale;
  const int first=std::max(0,k-10);for(int j=first;j<=k;++j)p.imu.push_back(samples[j]);a.poses.push_back(p);
 }
 return a;
}
int main(){int errors=0;
 auto run=[&](const char* name,MonoAlignment a,bool accepted){auto r=a.solve(lever);std::cout<<name<<' '<<r.accepted<<' '<<r.reason<<" scale="<<r.scale<<" ba="<<r.ba.transpose()<<" bg="<<r.bg.transpose()<<" excitation="<<r.excitation<<" residual="<<r.residual<<'\n';
  if(r.accepted!=accepted)++errors;
  if(accepted && (std::abs(r.scale-2.3)>.05 || (r.ba-ba).norm()>.1 || (r.bg-bg).norm()>.01))++errors;};
 run("exciting_motion",make(true,true,true),true);
 run("stationary",make(false,false,false),false);
 run("pure_rotation",make(false,true,false),false);
 // A fixed accelerometer bias rotating with the IMU must not manufacture
 // translational excitation. This bias is inside the configured 1m/s² bound.
 auto biased_rotation=make(false,true,false);
 auto biased_orientation=[](double t){return Eigen::Quaterniond(
     Eigen::AngleAxisd(.4*std::sin(t),V3::UnitX())*
     Eigen::AngleAxisd(.3*std::cos(.7*t),V3::UnitY())*
     Eigen::AngleAxisd(.7*t,V3::UnitZ()));};
 for(auto& p:biased_rotation.poses){p.q=biased_orientation(p.t);p.camera=p.q*lever/2.3;
  for(auto& sample:p.imu){const double t=sample.timestamp_;const auto q=biased_orientation(t);
   sample.angular_velocity_=logq(q.inverse()*biased_orientation(t+1e-5))/1e-5+bg;
   sample.linear_acceleration_=q.inverse()*(-g)+V3(.8,.2,.1);
  }
 }
 run("biased_pure_rotation",biased_rotation,false);
 run("constant_velocity",make(true,false,false),false);
 auto parallax=make(true,true,true);for(auto& p:parallax.poses)p.camera=V3::Zero();run("no_visual_parallax",parallax,false);
 auto gap=make(true,true,true);gap.poses[20].imu.erase(gap.poses[20].imu.begin()+2,gap.poses[20].imu.begin()+7);run("imu_gap",gap,false);
 auto repeated=make(true,true,true);repeated.poses[20].imu[2].timestamp_=repeated.poses[20].imu[1].timestamp_;run("repeated_imu",repeated,false);
 return errors?1:0;
}
