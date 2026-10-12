# Same-host CLOCK_MONOTONIC nanoseconds; paired offline protocol v1.
uint8 SYNC=0
uint8 START=1
uint8 HEARTBEAT=2
uint8 STOP=3
uint8 EMERGENCY=4
uint8 operation
string<=64 controller_session
string<=64 host_boot_id
string<=64 bridge_epoch
uint64 generation
uint64 sequence
int64 issued_ns
int64 start_ns
int64 deadline_ns
geometry_msgs/Twist velocity
