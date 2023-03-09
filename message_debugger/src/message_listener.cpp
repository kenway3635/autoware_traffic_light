
#include "ros_node.h"

void rosnode::timer_callback()
{
  double roi_percent = this->get_roi_info();
  std::cout<<"rough ROI percent is  "<<roi_percent<<"%"<<std::endl;
}

double rosnode::calculate_roi_size(const ROI a ,const ROI b)
{
  uint32_t rough_area = a.roi.height * a.roi.width;
  std::cout<<"rough area is " << rough_area << std::endl;
  uint32_t area = b.roi.height * b.roi.width;
  std::cout<<"area is "<<area<<std::endl;
  
  if(area != 0)
  {
    double area_percent = (rough_area-area)/area*100;
    return area_percent;
  }
  else
  {
    std::cerr<<"error, can not fetch roi from ssd_fine_detector"<<std::endl;
    return 0;
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rosnode>());
  rclcpp::shutdown();
  return 0;
}