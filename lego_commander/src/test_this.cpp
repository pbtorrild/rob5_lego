#include <math.h>
#include <ros/ros.h>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>

#include <message_filters/subscriber.h>

int main(int argc, char **argv) {
  //Ros init stuff:
  ros::init(argc, argv, "tf_sub");
  ros::NodeHandle nh;

  while (nh.ok()) {
    std::string del1 ="marker_";
    std::string del2 ="0";
    std::string sammen =del1+del2;
    ROS_INFO("sammen:%s",sammen.c_str());
    std::string uden = sammen.erase(0,7);
    ROS_INFO("uden  :%s",uden.c_str());
    int test = std::stoi(uden);
    ROS_INFO("test  :%d",test);
  }

  ros::spin();
}
