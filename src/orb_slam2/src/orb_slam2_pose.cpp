#include <cstdio>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "System.h"


using namespace std::chrono_literals;

class orb_slam2_pose: public rclcpp::Node
{
  public:
    orb_slam2_pose()
    : Node("orb_slam2_pose"), count_(0)
    {
      // publisher 
      pub_ = this->create_publisher<std_msgs::msg::String>("orb_slam2_pose", 10);

      // timer
      timer_ = this->create_wall_timer(
        500ms, std::bind(&orb_slam2_pose::timer_callback, this));
    }
    
    private:

      void timer_callback()
      {
        auto msg = std_msgs::msg::String();
        msg.data = "Publishing #" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());      
        pub_->publish(msg); 
      }

      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      rclcpp::TimerBase::SharedPtr timer_;
      size_t count_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<orb_slam2_pose>());
  rclcpp::shutdown();
  return 0;
}
