// Offline state-machine driver, never initializes ROS or actuator interfaces.
#include "command_watchdog.h"
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <memory>
int main(int argc, char **argv) {
  auto instance=std::make_unique<champ_guard::Watchdog>(argc>1?argv[1]:"controller-1", "host-1");
  auto &gate=*instance;
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream in(line); char kind; int64_t now; in>>kind>>now;
    bool accepted=false;
    if(kind=='R') { return 0; }
    else if(kind=='T') gate.tick(now);
    else if(kind=='Z') gate.legacy_twist({}, true, now);
    else if(kind=='B') gate.refuse_body_pose();
    else if(kind=='L') {
      double x,y,yaw; bool other_zero; in>>x>>y>>yaw>>other_zero;
      accepted=gate.legacy_twist({x,y,yaw},other_zero, now);
    }
    else if(kind=='P') {
      champ_guard::Packet p; unsigned op; std::string x,y,z;
      in>>op>>p.controller_session>>p.host_boot_id>>p.bridge_epoch>>p.generation>>p.sequence
        >>p.issued_ns>>p.start_ns>>p.deadline_ns>>x>>y>>z;
      p.operation=op; p.velocity={std::strtod(x.c_str(),nullptr),std::strtod(y.c_str(),nullptr),std::strtod(z.c_str(),nullptr)};
      accepted=gate.receive(p,now);
    }
    std::cout << "{\"protocol_version\":1,\"controller_session\":\""<<gate.controller_session
      <<"\",\"host_boot_id\":\""<<gate.host_boot_id<<"\",\"bridge_epoch\":\""<<gate.bridge_epoch
      <<"\",\"evaluated_ns\":"<<gate.evaluated_ns<<",\"generation\":"<<gate.generation
      <<",\"sequence\":"<<gate.sequence<<",\"synchronized\":"<<(gate.synchronized?"true":"false")
      <<",\"active\":"<<(gate.active?"true":"false")<<",\"gait_zero\":"<<(!gate.active?"true":"false")
      <<",\"deadline_ns\":"<<gate.deadline_ns<<",\"reason\":\""<<gate.reason
      <<"\",\"last_operation\":"<<unsigned(gate.last_request.operation)
      <<",\"last_request_generation\":"<<gate.last_request.generation
      <<",\"last_request_sequence\":"<<gate.last_request.sequence
      <<",\"last_request_issued_ns\":"<<gate.last_request.issued_ns
      <<",\"last_request_accepted\":"<<(gate.last_request_accepted?"true":"false")
      <<",\"accepted\":"<<(accepted?"true":"false")
      <<",\"linear_x\":"<<gate.velocity.x<<",\"linear_y\":"<<gate.velocity.y
      <<",\"angular_z\":"<<gate.velocity.yaw<<"}"<<std::endl;
  }
}
