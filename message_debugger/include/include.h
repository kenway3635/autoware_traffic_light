#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>

using std::placeholders::_1;
using autoware_auto_perception_msgs::msg::TrafficLightRoi;
