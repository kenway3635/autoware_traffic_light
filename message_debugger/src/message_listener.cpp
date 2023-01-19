
#include "include.h"

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber(): Node("minimal_subscriber")
    {
      //publisher
      //roi_area_pub= this->create_publisher<std_msgs::msg::Float32>("roi_percent",10);

      //subscriber
      rough_roi_sub = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>("rough/rois", 10, std::bind(&MinimalSubscriber::rough_roi_callback, this, _1));
      roi_sub = this->create_subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>("rois", 10, std::bind(&MinimalSubscriber::roi_callback, this, _1));

      //timer
      timer_ = this->create_wall_timer(500ms,std::bind(&MinimalSubscriber::timer_callback, this));
    }
    double get_roi_info() { return calculate_roi_size(rough_roi,roi); }



  private:

    void rough_roi_callback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::SharedPtr msg)  { rough_roi = msg->rois[0]; }
    void roi_callback(const autoware_auto_perception_msgs::msg::TrafficLightRoiArray::SharedPtr msg)  { roi = msg->rois[0]; }
    void timer_callback();


    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr rough_roi_sub;
    rclcpp::Subscription<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_sub;
    rclcpp::TimerBase::SharedPtr timer_;

    TrafficLightRoi rough_roi;
    TrafficLightRoi roi;
    double roi_area;
    double calculate_roi_size(const TrafficLightRoi a, const TrafficLightRoi b);
    autoware_auto_perception_msgs::msg::TrafficLight signal;
};


void MinimalSubscriber::timer_callback()
{
  double roi_percent = this->get_roi_info();
  std::cout<<"rough ROI percent is  "<<roi_percent<<"%"<<std::endl;
}
double MinimalSubscriber::calculate_roi_size(const TrafficLightRoi a ,const TrafficLightRoi b)
{
  uint32_t rough_area = a.roi.height * a.roi.width;
  std::cout<<"rough area is " << rough_area << std::endl;
  uint32_t area = b.roi.height * b.roi.width;
  std::cout<<"area is "<<area<<std::endl;
  
  double area_percent = (rough_area-area)/area*100;

  return area_percent;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}