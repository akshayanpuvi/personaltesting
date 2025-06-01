

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

void moveToStartPose(moveit::planning_interface::MoveGroupInterface& move_group) {
    std::vector<double> start_joints = {
        deg2rad(0.0),     // shoulder_pan
        deg2rad(-90.0),   // shoulder_lift
        deg2rad(0.0),     // elbow
        deg2rad(-90.0),   // wrist_1
        deg2rad(0.0),     // wrist_2
        deg2rad(0.0)      // wrist_3
    };

    move_group.setJointValueTarget(start_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Moving to start pose");
        move_group.execute(plan);
    } else {
        ROS_WARN("Failed to plan to start pose");
    }
    ros::Duration(2.0).sleep();
}

void movePoseToPose(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& pose) {
    move_group.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        ROS_INFO("Moving to new pose");
        move_group.execute(plan);
    } else {
        ROS_WARN("Failed to move to pose");
    }
    ros::Duration(1.5).sleep();
}

void addTable(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface) {
    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";

    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {0.8, 0.8, 0.1};

    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.0;
    table.primitive_poses[0].position.y = 0.35; //change this to 0.0 for origin rest
    table.primitive_poses[0].position.z = -0.05;
    table.primitive_poses[0].orientation.w = 1.0;

    table.operation = table.ADD;
    planning_scene_interface.applyCollisionObjects({table});
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test1_pick_place_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(10.0);
    group.setEndEffectorLink("gripper_tcp");

    addTable(planning_scene_interface);
    ros::Duration(1.0).sleep();

    moveToStartPose(group);

tf2::Quaternion orientation;
orientation.setRPY(M_PI, 0, -M_PI / 2);
geometry_msgs::Pose pose;
pose.orientation = tf2::toMsg(orientation);

pose.position.y = 0.25;
pose.position.z = 0.15;  // height above object

std::vector<double> x_positions = {-0.2, 0.2, -0.3, 0.3, -0.35, 0.35};

for (double x : x_positions) {
    pose.position.x = x;
    pose.position.z = 0.15;
    movePoseToPose(group, pose);  // above object

    pose.position.z = 0.05;
    movePoseToPose(group, pose);  // down to object

    pose.position.z = 0.15;
    movePoseToPose(group, pose);  // back up
}

moveToStartPose(group);  // Return home


    ros::waitForShutdown();
    return 0;
}










// INSERT THE BELOW AT LINE 79 - IF YOU WANT TO REVERT TO STANDARD XY TRNSLATION 

/* 

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI / 2);
    geometry_msgs::Pose pose;
    pose.orientation = tf2::toMsg(orientation);



    pose.position.x = 0.15;
    pose.position.y = 0.25;
    pose.position.z = 0.08;
    movePoseToPose(group, pose);

    pose.position.y = -0.25;
    movePoseToPose(group, pose);

    pose.position.x = -0.15;
    movePoseToPose(group, pose);



pose.position.y = 0.25;
pose.position.z = 0.08;

pose.position.x = -0.2;
movePoseToPose(group, pose);

pose.position.x = 0.2;
movePoseToPose(group, pose);

pose.position.x = -0.3;
movePoseToPose(group, pose);

pose.position.x = 0.3;
movePoseToPose(group, pose);

pose.position.x = -0.35;
movePoseToPose(group, pose);

pose.position.x = 0.35;
movePoseToPose(group, pose);


    moveToStartPose(group);

*/ // BOTTOM SHOULD REPLACE LINE 101
