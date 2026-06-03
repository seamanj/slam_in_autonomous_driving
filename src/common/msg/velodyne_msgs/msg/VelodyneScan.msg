# Velodyne LIDAR scan packets.
std_msgs/Header header           # standard ROS message header
velodyne_msgs/VelodynePacket[] packets    # vector of raw packets
monitor_msgs/FaultVec fault_vec           # error information (注意类型名大小写)