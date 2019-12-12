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

class tf_tracker{
private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  //the number of times a maker have to bee seen inorder for the avg to be taken
  int seen = 6;
  int num_markers=14;
  //Make a vector to take avg so that:
  //avg[id][0].x = tx
  //avg[id][1].x = rx
  std::vector<cv::Point3f> Point = decltype(Point)(2);
  std::vector<std::vector<cv::Point3f>> avg = decltype(avg)(num_markers,Point); //Create vector of ids containg a vector of size 4 each containing a cv point 3f
  std::vector<int> counter =decltype(counter)(num_markers);

protected:
  std::vector<geometry_msgs::Transform> avg_pos = decltype(avg_pos)(num_markers);
public:
  bool marker_found =false;

  tf_tracker():
  tf2_(buffer_),  target_frame_("world")
  {

  }

  ros::NodeHandle nh;
  void tracker(geometry_msgs::TransformStamped msg){
    //Find frame in regard to world

    geometry_msgs::TransformStamped frame;
    bool transform_succes;
    ros::Time stamp = msg.header.stamp;
    //wait to make sure pose is in the buffer
    ros::Duration(0.10).sleep();
    try{
       frame = buffer_.lookupTransform(msg.child_frame_id, "world",stamp);
       transform_succes=true;
    }
    catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
         transform_succes=false;
   }
   if (transform_succes==true) {
     //Get frame id as int
     std::string frame_id =msg.child_frame_id;
     frame_id.erase(0,7);
     int id_num = std::stoi(frame_id);
     //ser avg translation
     avg[id_num][0].x += frame.transform.translation.x;
     avg[id_num][0].y += frame.transform.translation.y;
     avg[id_num][0].z += frame.transform.translation.z;
     //set avg rotation
     double rx, ry, rz;
     tf2::Quaternion q(frame.transform.rotation.x,frame.transform.rotation.y,frame.transform.rotation.z,frame.transform.rotation.w);
     tf2::Matrix3x3 matrix(q);

     matrix.getRPY(rz,ry,rx);

     avg[id_num][1].x += rx;
     avg[id_num][1].y += ry;
     avg[id_num][1].z += rz;
     counter[id_num] += 1;

     if (counter[id_num]==seen) {


       //set avg trans
       float tx_avg=avg[id_num][0].x/seen;
       float ty_avg=avg[id_num][0].y/seen;
       float tz_avg=avg[id_num][0].z/seen;
       //set avg rotation
       float rx_avg=avg[id_num][1].x/seen;
       float ry_avg=avg[id_num][1].y/seen;
       float rz_avg=avg[id_num][1].z/seen;

       //Set avg pos

       avg_pos[id_num].translation.x = tx_avg;
       avg_pos[id_num].translation.y = ty_avg;
       avg_pos[id_num].translation.z = tz_avg;

       tf2::Quaternion Q;
       Q.setRPY(rx_avg,ry_avg,rz_avg);
       avg_pos[id_num].rotation.x = Q.x();
       avg_pos[id_num].rotation.y = Q.y();
       avg_pos[id_num].rotation.z = Q.z();
       avg_pos[id_num].rotation.w = Q.w();
       marker_found=true;
       //Reset all
       counter[id_num]=0;
       avg[id_num][0].x=0;
       avg[id_num][0].y=0;
       avg[id_num][0].z=0;
       //set avg rotation
       avg[id_num][1].x=0;
       avg[id_num][1].y=0;
       avg[id_num][1].z=0;
     }
   }
}


};

//The most helpfull sheet youll meet today :)
//http://docs.ros.org/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
int main(int argc, char **argv) {
  ros::init(argc, argv, "Commander");
  //Define class intace
  tf_tracker instance;

  ros::Subscriber sub = instance.nh.subscribe("tf/marker_frames", 10, &tf_tracker::tracker,&instance);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::PlanningSceneInterface setEndEffectorLink("ee_link"); //NOtE: THIS CAN BE ANY FRAME ON THE ENDEFFECTOR
  move_group.setPoseReferenceFrame("world");

  move_group.startStateMonitor();
  /*Seach for markers */
  std::vector<double> joint_group_positions =decltype(joint_group_positions)(6);
  moveit::planning_interface::MoveGroupInterface::Plan search;

  while (instance.marker_found==false && ros::ok()) {

    joint_group_positions[0] = 0.001;
    joint_group_positions[1] = -M_PI/2;
    joint_group_positions[2] = -M_PI/2;
    joint_group_positions[3] = -M_PI/4;
    joint_group_positions[4] = M_PI/2;
    joint_group_positions[5] = 0;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
    joint_group_positions[0] = 2*M_PI-0.001;
    move_group.setJointValueTarget(joint_group_positions);
    move_group.plan(search);
    move_group.move();
  }
  //HOW TO ACCES TF DATA FOR THE MARKE IR2 WORLD
  //geometry_msgs::Transform Goal = instance.avg_pos[marker_id];

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
