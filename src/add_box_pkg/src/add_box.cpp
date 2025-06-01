#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
#include <geometry_msgs/Pose.h>



// Add both boxes to the planning scene
void addCollisionBoxesToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(1);  // Two objects


  collision_objects[0].id = "separator";
  collision_objects[0].header.frame_id = "base_link";

  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_objects[0].primitives[0].dimensions = {0.02, 0.2, 0.08};  // x, y, z

  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].orientation.w = 1.0;
  collision_objects[0].primitive_poses[0].position.x = 0.0;
  collision_objects[0].primitive_poses[0].position.y = 0.35;
  collision_objects[0].primitive_poses[0].position.z = 0.05;  // Half height above table

  collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;

  // Apply both
  planning_scene_interface.applyCollisionObjects(collision_objects);
  ROS_INFO("Added 'box1' and 'separator' collision objects to the planning scene.");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_box_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Duration(2.0).sleep();  // Wait for ROS and MoveIt to initialize

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionBoxesToScene(planning_scene_interface);

  ros::Duration(3.0).sleep();  // Allow time for visualization in RViz

  ros::shutdown();
  return 0;
}


















// #include <ros/ros.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <shape_msgs/SolidPrimitive.h>
// #include <geometry_msgs/Pose.h>



// // Add both boxes to the planning scene
// void addCollisionBoxesToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.resize(2);  // Two objects

//   // === Box 1 ===
//   collision_objects[0].id = "box1";
//   collision_objects[0].header.frame_id = "base_link";

//   collision_objects[0].primitives.resize(1);
//   collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//   collision_objects[0].primitives[0].dimensions = {0.5, 0.5, 0.1};  // x, y, z

//   collision_objects[0].primitive_poses.resize(1);
//   collision_objects[0].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[0].primitive_poses[0].position.x = 0.0;
//   collision_objects[0].primitive_poses[0].position.y = 0.2;
//   collision_objects[0].primitive_poses[0].position.z = -0.06;  // Half below table

//   collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;

//   // === Separator Box ===
//   collision_objects[1].id = "separator";
//   collision_objects[1].header.frame_id = "base_link";

//   collision_objects[1].primitives.resize(1);
//   collision_objects[1].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//   collision_objects[1].primitives[0].dimensions = {0.02, 0.2, 0.1};  // x, y, z

//   collision_objects[1].primitive_poses.resize(1);
//   collision_objects[1].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[1].primitive_poses[0].position.x = 0.0;
//   collision_objects[1].primitive_poses[0].position.y = 0.35;
//   collision_objects[1].primitive_poses[0].position.z = 0.1;  // Half height above table

//   collision_objects[1].operation = moveit_msgs::CollisionObject::ADD;

//   // Apply both
//   planning_scene_interface.applyCollisionObjects(collision_objects);
//   ROS_INFO("Added 'box1' and 'separator' collision objects to the planning scene.");
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "add_box_node");
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   ros::Duration(2.0).sleep();  // Wait for ROS and MoveIt to initialize

//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   addCollisionBoxesToScene(planning_scene_interface);

//   ros::Duration(3.0).sleep();  // Allow time for visualization in RViz

//   ros::shutdown();
//   return 0;
// }
















// #include <ros/ros.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>
// #include <moveit_msgs/CollisionObject.h>
// #include <shape_msgs/SolidPrimitive.h>
// #include <geometry_msgs/Pose.h>


// // 0.2 meter wall seperator

// void addCollisionBoxToScene(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.resize(1);  // Only one object

//   // Set object ID and reference frame
//   collision_objects[0].id = "box1";
//   collision_objects[0].header.frame_id = "base_link";

//   // Define the box primitive
//   collision_objects[0].primitives.resize(1);
//   collision_objects[0].primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//   collision_objects[0].primitives[0].dimensions.resize(3);
//   collision_objects[0].primitives[0].dimensions[0] = 0.5;  // x
//   collision_objects[0].primitives[0].dimensions[1] = 0.5;  // y
//   collision_objects[0].primitives[0].dimensions[2] = 0.1;  // z

//   // Define the pose of the box
//   collision_objects[0].primitive_poses.resize(1);
//   collision_objects[0].primitive_poses[0].orientation.w = 1.0;
//   collision_objects[0].primitive_poses[0].position.x = 0.0;
//   collision_objects[0].primitive_poses[0].position.y = 0.2;
//   collision_objects[0].primitive_poses[0].position.z = -0.06;  // Half height

//   // Mark it as an addition to the scene
//   collision_objects[0].operation = moveit_msgs::CollisionObject::ADD;

//   // Apply to the planning scene
//   planning_scene_interface.applyCollisionObjects(collision_objects);

//   ROS_INFO("Added box1 collision object to the planning scene.");
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "add_box_node");
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   ros::Duration(2.0).sleep();  // Wait for MoveIt and ROS to initialize

//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   addCollisionBoxToScene(planning_scene_interface);

//   ros::Duration(3.0).sleep();  // Give some time to visualize in RViz

//   ros::shutdown();
//   return 0;
// }
