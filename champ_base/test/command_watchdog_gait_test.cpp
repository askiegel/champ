// Offline core gait/trajectory comparison. No ROS initialization or actuator I/O.
#include "command_watchdog.h"
#include <champ/leg_controller/leg_controller.h>
#include <iostream>
#include <stdexcept>
#include <cstdlib>
void check(bool ok) { if(!ok) throw std::runtime_error("gait comparison failed"); }
int main(int argc, char **argv) {
  check(argc==4);
  champ::GaitConfig config;
  config.max_linear_velocity_x=.5; config.max_linear_velocity_y=.1;
  config.max_angular_velocity_z=1.; config.stance_duration=.3;
  config.swing_height=.02; config.stance_depth=.005;
  champ::QuadrupedBase guarded(config), original(config);
  for(auto *base:{&guarded,&original}) for(int i=0;i<4;i++) {
    base->legs[i]->hip.setTranslation(i<2?.12f:-.12f, i%2?.1f:-.1f, 0);
    base->legs[i]->lower_leg.setTranslation(0,0,-.1f);
    base->legs[i]->foot.setTranslation(0,0,-.1f);
  }
  champ::LegController actual(guarded,1000000), baseline(original,1000000);
  champ_guard::Watchdog gate("controller","host");
  champ_guard::Packet p;
  p.controller_session="controller";p.host_boot_id="host";p.bridge_epoch="bridge";
  p.issued_ns=1000000000;check(gate.receive(p,p.issued_ns));
  p.operation=champ_guard::Start;p.generation=1;p.sequence=1;
  p.issued_ns+=1;p.start_ns=p.issued_ns;p.deadline_ns=p.start_ns+2000000000;
  p.velocity={std::strtod(argv[1],nullptr),std::strtod(argv[2],nullptr),std::strtod(argv[3],nullptr)};
  check(gate.receive(p,p.issued_ns));
  for(int t=0;t<200000;t+=5000) {
    gate.tick(p.start_ns+int64_t(t)*1000);
    champ::Velocities current, unchanged;
    current.linear.x=gate.velocity.x;current.linear.y=gate.velocity.y;current.angular.z=gate.velocity.yaw;
    unchanged.linear.x=p.velocity.x;unchanged.linear.y=p.velocity.y;unchanged.angular.z=p.velocity.yaw;
    geometry::Transformation a[4], b[4];
    for(int i=0;i<4;i++) { a[i]=guarded.legs[i]->zero_stance();b[i]=original.legs[i]->zero_stance(); }
    actual.velocityCommand(a,current,1000000+t);baseline.velocityCommand(b,unchanged,1000000+t);
    for(int i=0;i<4;i++) {
      check(std::isfinite(a[i].X()) && std::isfinite(a[i].Y()) && std::isfinite(a[i].Z()));
      check(a[i].X()==b[i].X() && a[i].Y()==b[i].Y() && a[i].Z()==b[i].Z());
    }
  }
  gate.tick(p.start_ns+champ_guard::kReceiptTimeoutNs);check(!gate.active);
  champ::Velocities zero;
  geometry::Transformation stopped[4];
  for(int i=0;i<4;i++) stopped[i]=guarded.legs[i]->zero_stance();
  actual.velocityCommand(stopped,zero,1250000);
  for(int i=0;i<4;i++) {
    auto stance=guarded.legs[i]->zero_stance();
    check(stopped[i].X()==stance.X() && stopped[i].Y()==stance.Y() && stopped[i].Z()==stance.Z());
    check(actual.phase_generator.swing_phase_signal[i]==0 && actual.phase_generator.stance_phase_signal[i]==0);
  }
  std::cout<<"unchanged valid-command trajectories; expiry returns zero gait; no torque operation\n";
}
