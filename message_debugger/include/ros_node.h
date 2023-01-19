#ifndef ROS_NODE
#define ROS_NODE

#include <memory>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_signal.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

typedef autoware_auto_perception_msgs::msg::TrafficLightRoi ROI;
typedef autoware_auto_perception_msgs::msg::TrafficLightRoiArray ROIArray;

class rosnode : public rclcpp::Node
{
  public:
    rosnode(): Node("traffic_message_debugger")
    {
      //publisher
      //roi_area_pub= this->create_publisher<std_msgs::msg::Float32>("roi_percent",10);

      //subscriber
      rough_roi_sub = this->create_subscription<ROIArray>("rough/rois", 10, std::bind(&rosnode::rough_roi_callback, this, _1));
      roi_sub = this->create_subscription<ROIArray>("rois", 10, std::bind(&rosnode::roi_callback, this, _1));

      //timer
      timer_ = this->create_wall_timer(500ms,std::bind(&rosnode::timer_callback, this));
    }
    
    double get_roi_info() { return calculate_roi_size(rough_roi,roi); }



  private:

    void rough_roi_callback(const ROIArray::SharedPtr msg)  { rough_roi = msg->rois[0]; }
    void roi_callback(const ROIArray::SharedPtr msg)  { roi = msg->rois[0]; }
    void timer_callback();


    rclcpp::Subscription<ROIArray>::SharedPtr rough_roi_sub;
    rclcpp::Subscription<ROIArray>::SharedPtr roi_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    ROI rough_roi;
    ROI roi;
    double roi_area;
    double calculate_roi_size(const ROI a, const ROI b);
    autoware_auto_perception_msgs::msg::TrafficLight signal;
};

#endif