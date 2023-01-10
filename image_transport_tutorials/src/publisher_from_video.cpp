// Copyright 2021, Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi.hpp>

#include <yaml-cpp/yaml.h>
#include <string.h>

//usage:ros2 run image_transport_tutorials publisher_from_video "PATH_TO_FILE"


int main(int argc, char *argv[])
{
  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) {
    std::cout<<"need to specify a video file"<<std::endl;
    return 1;
  }

  //read config file with topic name & manually tuned roi
  
  std::string yaml_path = std::string(FILE_PATH)+"/param/config.yaml";
  YAML::Node config = YAML::LoadFile(yaml_path);

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher", options);


  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise(config["image_topic_name"].as<std::string>(), 1);
  auto roi_pub_ = node->create_publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>(config["roi"]["topic_name"].as<std::string>(), 1);
  
 

  // Convert the command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);

  
  //open video file
  cv::VideoCapture cap(argv[1]);

  // Check if video device can be opened with the given index
  if (!cap.isOpened()) {
    std::cout<<"can not open the video file"<<std::endl;
    return 1;
  }
  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  autoware_auto_perception_msgs::msg::TrafficLightRoi tl_roi;
  autoware_auto_perception_msgs::msg::TrafficLightRoiArray output_msg;
  
  std::string name;
  for (char *filename = argv[1]; strcmp(filename,".mp4")!=0; filename++)
  {
    if (*filename == '/')
    {
      name = "";
      continue;
    }
    name.push_back(*filename);
  }
  std::cout<<"choose file : "<< name<<std::endl;


  tl_roi.id = 1;
  tl_roi.roi.x_offset=config["roi"][name]["x_offset"].as<int>();
  tl_roi.roi.y_offset=config["roi"][name]["y_offset"].as<int>();
  tl_roi.roi.height = config["roi"][name]["height"].as<int>();
  tl_roi.roi.width = config["roi"][name]["width"].as<int>();
  output_msg.rois.push_back(tl_roi);
  output_msg.header = hdr;

  rclcpp::WallRate loop_rate(config["looprate"].as<int>());
  std::cout<<"publishing..."<<std::endl;
  while (rclcpp::ok()) {
    
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if (!frame.empty()) {
      cv::resize(frame, frame, cv::Size(1280,720), 0, 0,CV_INTER_LINEAR);
      msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
      pub.publish(msg);
      roi_pub_->publish(output_msg);
      //cv::imshow("video_stream",frame);
      //cv::waitKey(10);
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
