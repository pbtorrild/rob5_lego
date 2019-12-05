#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <math.h>

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("gripper_body"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("world");
  move_group.startStateMonitor();
  /*Seach for markers */
  //im think add code that makes the first joint go from -pi to pi

  while (ros::ok()) {
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = -0.3;
    target_pose.position.z = 0.2;
    tf2::Quaternion q;
    q.setRPY(-M_PI/2,0,-M_PI/2);
    target_pose.orientation.x=q.x();
    target_pose.orientation.y=q.y();
    target_pose.orientation.z=q.z();
    target_pose.orientation.w=q.w();
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.plan(my_plan);
    move_group.move();
    sleep(3.0);
    target_pose.position.x = 0.2;
    target_pose.position.y = -0.5;
    target_pose.position.z = 0.4;

    q.setRPY(-M_PI/2, -M_PI,M_PI/4);
    target_pose.orientation.x=q.x();
    target_pose.orientation.y=q.y();
    target_pose.orientation.z=q.z();
    target_pose.orientation.w=q.w();
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
    move_group.plan(my_plan2);
    move_group.move();
    sleep(3.0);
  }

  ros::spin();
}
