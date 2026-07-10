#pragma once

#include <memory>

#include "velodyne_msgs/msg/velodyne_packet.hpp"
#include "velodyne_msgs/msg/velodyne_scan.hpp"

using PacketMsg = velodyne_msgs::msg::VelodynePacket;
using PacketMsgPtr = std::shared_ptr<PacketMsg>;

using PacketsMsg = velodyne_msgs::msg::VelodyneScan;
using PacketsMsgPtr = std::shared_ptr<PacketsMsg>;