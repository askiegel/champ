// Controller-local receipt watchdog. No ROS, actuator, or transport I/O.
#pragma once
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <utility>

namespace champ_guard {
constexpr int64_t kReceiptTimeoutNs = 250000000; // OFFLINE validation target.
constexpr int64_t kMaximumDurationNs = 2000000000; // Existing Bridge ceiling.
enum Operation : uint8_t { Sync=0, Start=1, Heartbeat=2, Stop=3, Emergency=4 };
struct Velocity { double x=0, y=0, yaw=0; };
struct Packet {
  uint8_t operation=Sync;
  std::string controller_session, host_boot_id, bridge_epoch;
  uint64_t generation=0, sequence=0;
  int64_t issued_ns=0, start_ns=0, deadline_ns=0;
  Velocity velocity;
};
class Watchdog {
 public:
  const std::string controller_session, host_boot_id;
  std::string bridge_epoch, reason="not_synchronized";
  uint64_t generation=0, sequence=0;
  int64_t deadline_ns=0, evaluated_ns=0;
  bool synchronized=false, active=false;
  Packet last_request;
  bool last_request_accepted=false;
  Velocity velocity;
  Watchdog(std::string session, std::string boot)
      : controller_session(std::move(session)), host_boot_id(std::move(boot)) {}
  void zero(const std::string &why) {
    active=false; velocity={}; reason=why; revoked_at_ns_=evaluated_ns; // Generation floor is never reset.
  }
  void tick(int64_t now) {
    if(now<0 || now<evaluated_ns) { zero("monotonic_clock_regressed"); fault_=true; return; }
    evaluated_ns=now;
    if(active) {
      const bool receipt_first=deadline_ns-issued_ns_>kReceiptTimeoutNs;
      // Report the first expired bound, even when evaluation was delayed.
      if((now-issued_ns_>=kReceiptTimeoutNs || now-received_ns_>=kReceiptTimeoutNs) && receipt_first) zero("command_receipt_expired");
      else if(now>=deadline_ns) zero("command_deadline_expired");
    }
  }
  bool legacy_twist(const Velocity &v, bool other_axes_zero, int64_t now) {
    tick(now);
    if(is_zero(v) && other_axes_zero) { zero("legacy_emergency_gait_zero"); return true; }
    reason="legacy_nonzero_twist_refused"; return false;
  }
  void refuse_body_pose() { reason="independent_body_pose_refused"; }
  bool receive(const Packet &p, int64_t now) {
    tick(now); last_request=p; last_request_accepted=false;
    auto refuse=[this](const char *why) { reason=why; return false; };
    if(fault_) return refuse("monotonic_clock_fault");
    if(p.controller_session!=controller_session || p.host_boot_id!=host_boot_id)
      return refuse("controller_or_host_session_mismatch");
    if(p.operation==Emergency) {
      if(p.bridge_epoch==bridge_epoch && p.generation>generation) generation=p.generation;
      if(generation>highest_generation_) highest_generation_=generation;
      zero("emergency_gait_zero"); return last_request_accepted=true;
    }
    if(!identifier(p.bridge_epoch) || p.issued_ns<0 || p.issued_ns>now ||
       now-p.issued_ns>=kReceiptTimeoutNs) return refuse("invalid_or_expired_packet");
    if(p.operation==Sync) {
      if(active) return refuse("sync_while_active");
      if(p.generation || p.sequence || p.start_ns || p.deadline_ns || !is_zero(p.velocity))
        return refuse("invalid_sync");
      if(retired_epochs_.count(p.bridge_epoch)) return refuse("retired_bridge_epoch");
      if(p.bridge_epoch!=bridge_epoch) {
        if(retired_epochs_.size()>=64) return refuse("epoch_history_exhausted");
        if(!bridge_epoch.empty()) retired_epochs_.insert(bridge_epoch);
        bridge_epoch=p.bridge_epoch; generation=sequence=highest_generation_=0;
      }
      synchronized=true; zero("synchronized_zero"); return last_request_accepted=true;
    }
    if(!synchronized || p.bridge_epoch!=bridge_epoch) return refuse("bridge_epoch_mismatch");
    if(p.operation==Stop) {
      if(p.generation==0 || p.sequence==0 || p.generation<generation ||
         (active && p.generation!=generation) ||
         (p.generation==generation && p.sequence<=sequence) || !is_zero(p.velocity))
        return refuse("stale_or_invalid_scoped_stop");
      generation=p.generation;
      if(generation>highest_generation_) highest_generation_=generation;
      sequence=p.sequence; zero("scoped_gait_zero"); return last_request_accepted=true;
    }
    if(p.operation!=Start && p.operation!=Heartbeat) return refuse("invalid_operation");
    // An attempted START is single-use even when refused while another command
    // is active. It cannot turn into a delayed dispatch after that command stops.
    if(p.operation==Start && p.generation>0) {
      if(p.generation<=highest_generation_) return refuse("retired_or_duplicate_start");
      highest_generation_=p.generation;
    }
    if(p.generation==0 || p.sequence==0 || !valid(p.velocity) || is_zero(p.velocity) ||
       p.start_ns<0 || p.start_ns>p.issued_ns || p.deadline_ns<=p.start_ns ||
       p.deadline_ns-p.start_ns>kMaximumDurationNs || now>=p.deadline_ns) {
      if(p.generation==generation) zero("invalid_active_command");
      return refuse("invalid_command_or_deadline");
    }
    if(p.operation==Start) {
      if(active) return refuse("command_owner_busy");
      if(p.start_ns<=revoked_at_ns_) return refuse("command_predates_revocation");
      if(p.generation<=generation || p.sequence!=1) return refuse("retired_or_invalid_generation");
      generation=p.generation; sequence=1; start_ns_=p.start_ns; deadline_ns=p.deadline_ns;
      velocity=p.velocity; active=true;
    } else {
      if(!active || p.generation!=generation) return refuse("retired_or_unknown_generation");
      if(p.sequence<=sequence) return refuse("duplicate_or_regressed_sequence");
      if(p.start_ns!=start_ns_ || p.deadline_ns!=deadline_ns ||
         p.velocity.x!=velocity.x || p.velocity.y!=velocity.y || p.velocity.yaw!=velocity.yaw ||
         p.issued_ns<=issued_ns_) {
        zero("heartbeat_mutated_command"); return refuse("heartbeat_mutated_command");
      }
      sequence=p.sequence;
    }
    issued_ns_=p.issued_ns; received_ns_=now; reason="command_active";
    return last_request_accepted=true;
  }
  static bool is_zero(const Velocity &v) { return v.x==0 && v.y==0 && v.yaw==0; }
 private:
  bool fault_=false;
  uint64_t highest_generation_=0;
  int64_t issued_ns_=0, received_ns_=0, start_ns_=0, revoked_at_ns_=0;
  std::set<std::string> retired_epochs_;
  static bool identifier(const std::string &s) {
    if(s.empty() || s.size()>64) return false;
    for(char c:s) if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-')) return false;
    return true;
  }
  static bool valid(const Velocity &v) {
    return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.yaw) &&
      std::abs(v.x)<=.50 && std::abs(v.y)<=.10 && std::abs(v.yaw)<=1.0 &&
      ((v.x!=0)+(v.y!=0)+(v.yaw!=0))==1;
  }
};
} // namespace champ_guard
