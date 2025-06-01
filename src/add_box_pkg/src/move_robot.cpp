#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>


// Constants
static const std::string MANIPULATOR_GROUP = "manipulator";
static const std::string ONROBOT_GROUP = "onrobot";

// Global interfaces
moveit::planning_interface::MoveGroupInterface* manipulator_move_group_ptr;
moveit::planning_interface::MoveGroupInterface* onrobot_move_group_ptr;

// Constant orientation for all poses
geometry_msgs::Quaternion getFixedOrientation()
{
  geometry_msgs::Quaternion q;
  q.x = 0.9999999928689334;
  q.y = -8.242325877950169e-05;
  q.z = -4.394133294589832e-05;
  q.w = 7.441571972498993e-05;
  return q;

  // [0.9999999928689334, -8.242325877950169e-05, -4.394133294589832e-05, 7.441571972498993e-05]

}





// Add both boxes to the planning scene
void addCollisionBoxesToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);  // Two objects

  // === Box 1 ===
  collision_objects[0].id = "box1";
  collision_objects[0].header.frame_id = "base_link";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions = {0.5, 0.5, 0.1};  // x, y, z

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.2;
  collision_objects[0].primitive_poses[0].position.z = -0.06;  // Half below table

  collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;


  // Apply both
  planning_scene_interface.applyCollisionObjects(collision_objects);
  ROS_INFO("Added 'box1' and 'separator' collision objects to the planning scene.");
}










// Move to specified XYZ with fixed orientation
bool moveToXYZ(double x, double y, double z)
{
  geometry_msgs::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation = getFixedOrientation();

  manipulator_move_group_ptr->setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (manipulator_move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO("Moving to [%.3f, %.3f, %.3f]", x, y, z);
    manipulator_move_group_ptr->execute(plan);
  }
  else
  {
    ROS_ERROR("Planning failed to [%.3f, %.3f, %.3f]", x, y, z);
  }

  return success;
}

// Open or close the gripper
bool controlGripper(const std::string& command)
{
  if (command != "open" && command != "close")
  {
    ROS_ERROR("Gripper command must be 'open' or 'close'");
    return false;
  }

  onrobot_move_group_ptr->setNamedTarget(command);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (onrobot_move_group_ptr->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    ROS_INFO("Gripper '%s' command executing...", command.c_str());
    onrobot_move_group_ptr->execute(plan);
  }
  else
  {
    ROS_ERROR("Gripper '%s' planning failed.", command.c_str());
  }

  return success;
}

// Pick and place routine
// void pickAndPlace(const std::vector<double>& pick_xyz, const std::vector<double>& place_xyz)
// {
//   moveToXYZ(pick_xyz[0], pick_xyz[1], pick_xyz[2]);
//   controlGripper("close");
//   ros::Duration(2.0).sleep();

//   moveToXYZ(place_xyz[0], place_xyz[1], place_xyz[2]);
//   controlGripper("open");
//   ros::Duration(2.0).sleep();
// }




void pickAndPlace(const std::vector<double>& pick_xyz, const std::vector<double>& place_xyz)
{
  // Move to 'ready' first before picking
  manipulator_move_group_ptr->setNamedTarget("ready");
  moveit::planning_interface::MoveGroupInterface::Plan to_ready_before_pick;
  if (manipulator_move_group_ptr->plan(to_ready_before_pick) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    manipulator_move_group_ptr->execute(to_ready_before_pick);
    ROS_INFO("Moved to 'ready' before picking.");
  }

  moveToXYZ(pick_xyz[0], pick_xyz[1], pick_xyz[2]);
  controlGripper("close");
  ros::Duration(2.0).sleep();

  // Move to 'ready' before placing
  manipulator_move_group_ptr->setNamedTarget("ready");
  moveit::planning_interface::MoveGroupInterface::Plan to_ready_before_place;
  if (manipulator_move_group_ptr->plan(to_ready_before_place) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    manipulator_move_group_ptr->execute(to_ready_before_place);
    ROS_INFO("Moved to 'ready' before placing.");
  }

  moveToXYZ(place_xyz[0], place_xyz[1], place_xyz[2]);
  controlGripper("open");
  ros::Duration(2.0).sleep();
}











int main(int argc, char** argv)
{
  ros::init(argc, argv, "batch_pick_and_place");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionBoxesToScene(planning_scene_interface);

  ros::Duration(3.0).sleep();  // Allow time for visualization in RViz

  moveit::planning_interface::MoveGroupInterface manipulator_move_group(MANIPULATOR_GROUP);
  moveit::planning_interface::MoveGroupInterface onrobot_move_group(ONROBOT_GROUP);
  manipulator_move_group.setPoseReferenceFrame("base_link");

  // Assign to global pointers
  manipulator_move_group_ptr = &manipulator_move_group;
  onrobot_move_group_ptr = &onrobot_move_group;

  // Define pick poses (X, Y, Z)
  std::vector<std::vector<double>> pick_poses = {
    {-0.143, 0.324, 0.029},  // pose1
    {-0.2056, 0.372, 0.029},  // pose2
    {-0.155, 0.381, 0.029}   // pose3
  };

  // Define the fixed place pose
  std::vector<double> place_pose = {0.175, 0.313, 0.029};

  // Loop through all pick poses and place
  for (const auto& pick_pose : pick_poses)
  {
    pickAndPlace(pick_pose, place_pose);
  }

  // Final move to "ready" named position
  manipulator_move_group.setNamedTarget("ready");
  moveit::planning_interface::MoveGroupInterface::Plan ready_plan;
  if (manipulator_move_group.plan(ready_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
  {
    manipulator_move_group.execute(ready_plan);
    ROS_INFO("Returned to 'ready' position.");
  }
  else
  {
    ROS_WARN("Failed to move to 'ready' position.");
  }

  ros::shutdown();
  return 0;
}









// #include <ros/ros.h>
// #include <moveit/move_group_interface/move_group_interface.h>



// // pose1
// // [-0.143, 0.324, 0.029]

// // pose2
// // [-0.056, 0.372, 0.029]

// // pose3
// // [-0.155, 0.381, 0.029]

// // place pose 

// // [0.175, 0.313, 0.029]






// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "move_to_gripper_tcp_pose_and_close");
//   ros::NodeHandle node_handle;
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   // Move group for manipulator to move to target pose
//   static const std::string MANIPULATOR_GROUP = "manipulator";
//   moveit::planning_interface::MoveGroupInterface manipulator_move_group(MANIPULATOR_GROUP);
//   manipulator_move_group.setPoseReferenceFrame("base_link");

//   // First target pose for manipulator
//   geometry_msgs::Pose target_pose;
//   target_pose.position.x = 0.006;
//   target_pose.position.y = 0.437;
//   target_pose.position.z = 0.119;
//   target_pose.orientation.x = 0.0;
//   target_pose.orientation.y = 1.0;
//   target_pose.orientation.z = 0.0;
//   target_pose.orientation.w = 0.0;

//   manipulator_move_group.setPoseTarget(target_pose);

//   moveit::planning_interface::MoveGroupInterface::Plan manipulator_plan;
//   bool success = (manipulator_move_group.plan(manipulator_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (success)
//   {
//     ROS_INFO("Manipulator planning successful. Executing the plan...");
//     manipulator_move_group.execute(manipulator_plan);
//     ROS_INFO("Manipulator motion executed.");
//   }
//   else
//   {
//     ROS_ERROR("Manipulator failed to plan motion.");
//   }

//   // MoveGroup for onrobot (gripper)
//   static const std::string ONROBOT_GROUP = "onrobot";
//   moveit::planning_interface::MoveGroupInterface onrobot_move_group(ONROBOT_GROUP);

//   // Close gripper
//   onrobot_move_group.setNamedTarget("close");
//   moveit::planning_interface::MoveGroupInterface::Plan onrobot_plan;
//   success = (onrobot_move_group.plan(onrobot_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (success)
//   {
//     ROS_INFO("Onrobot 'close' named target plan successful. Executing...");
//     onrobot_move_group.execute(onrobot_plan);
//     ROS_INFO("Onrobot motion executed.");
//   }
//   else
//   {
//     ROS_ERROR("Onrobot failed to plan for 'close' named target.");
//   }

//   ros::Duration(2.0).sleep();  // Wait for gripper to close

//   // Move manipulator to second pose
//   target_pose.position.x = 0.400;
//   target_pose.position.y = 0.092;
//   target_pose.position.z = 0.174;
//   target_pose.orientation.x = 0.0;
//   target_pose.orientation.y = 1.0;
//   target_pose.orientation.z = 0.0;
//   target_pose.orientation.w = 0.0;

//   manipulator_move_group.setPoseTarget(target_pose);
//   success = (manipulator_move_group.plan(manipulator_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

//   if (success)
//   {
//     ROS_INFO("Manipulator planning successful. Executing the plan...");
//     manipulator_move_group.execute(manipulator_plan);
//     ROS_INFO("Manipulator motion executed.");
//   }
//   else
//   {
//     ROS_ERROR("Manipulator failed to plan motion.");
//   }

//   // Open gripper at the end
//   onrobot_move_group.setNamedTarget("open");
//   success = (onrobot_move_group.plan(onrobot_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   if (success)
//   {
//     ROS_INFO("Onrobot 'open' named target plan successful. Executing...");
//     onrobot_move_group.execute(onrobot_plan);
//     ROS_INFO("Onrobot motion executed.");
//   }


//   ros::Duration(2.0).sleep();
//   // Move manipulator to predefined "ready" position at the end
//   manipulator_move_group.setNamedTarget("ready");
//   success = (manipulator_move_group.plan(manipulator_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   if (success)
//   {
//     ROS_INFO("Manipulator moving to 'ready' position...");
//     manipulator_move_group.execute(manipulator_plan);
//     ROS_INFO("Manipulator reached 'ready' position.");
//   }
//   else
//   {
//     ROS_ERROR("Failed to plan manipulator 'ready' position.");
//   }

//   ros::shutdown();
//   return 0;
// }
