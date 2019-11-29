#include <moveit/move_group_interface/move_group_interface.h>
int main(int argc, char const *argv[]) {
  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  //move_group.setPoseReferenceFrame(base_frame);
  //move_group.setPoseTarget(move_target);
  //move_group.move();
  ros::spin();
}
