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

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      rough_roi_sub = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>("rough/rois", 10, std::bind(&MinimalSubscriber::rough_roi_callback, this, _1));
      roi_sub = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>("rois", 10, std::bind(&MinimalSubscriber::roi_callback, this, _1));
    }
    double get_roi_info()
    {
      return calculate_roi_size(rough_roi,roi);
    }



  private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    void rough_roi_callback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::SharedPtr msg)
    {
      rough_roi = msg->rois[0];
    }
    void roi_callback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::SharedPtr msg)
    {
      roi = msg->rois[0];
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr rough_roi_sub;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_sub;

    TrafficLightRoi rough_roi;
    TrafficLightRoi roi;
    double roi_area;
    double calculate_roi_size(const TrafficLightRoi a, const TrafficLightRoi b);
    autoware_auto_perception_msgs::msg::TrafficLight signal;
};

double MinimalSubscriber::calculate_roi_size(const TrafficLightRoi a ,const TrafficLightRoi b)
{
  uint32_t rough_area = a.roi.height * a.roi.width;
  uint32_t area = b.roi.height * b.roi.width;
  double area_percent = (rough_area-area)/area;

  return area_percent;
};