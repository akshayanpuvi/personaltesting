#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Trigger.h>

inline double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

geometry_msgs::Pose lifted(const geometry_msgs::Pose& pose, double dz) {
    geometry_msgs::Pose p = pose;
    p.position.z += dz;
    return p;
}

void callGripperService(const std::string& service_name) {
    ros::ServiceClient client = ros::NodeHandle().serviceClient<std_srvs::Trigger>(service_name);
    std_srvs::Trigger srv;
    if (client.call(srv)) {
        ROS_INFO_STREAM("Gripper service call to '" << service_name << "' succeeded: " << srv.response.message);
    } else {
        ROS_WARN_STREAM("Failed to call gripper service: " << service_name);
    }
    ros::Duration(1.0).sleep();
}

void moveCartesian(moveit::planning_interface::MoveGroupInterface& group, const std::vector<geometry_msgs::Pose>& waypoints) {
    moveit_msgs::RobotTrajectory trajectory;
    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0.95) {
        ROS_INFO("Planned path %.2f%%. Executing...", fraction * 100.0);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        group.execute(plan);
    } else {
        ROS_WARN("Path planning incomplete %.2f%%. Skipping execution.", fraction * 100.0);
    }
    ros::Duration(1.0).sleep();
}

void addTable(moveit::planning_interface::PlanningSceneInterface& psi) {
    moveit_msgs::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "base_link";

    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions = {0.8, 0.7, 0.1};

    table.primitive_poses.resize(1);
    table.primitive_poses[0].position.x = 0.0;
    table.primitive_poses[0].position.y = 0.23;
    table.primitive_poses[0].position.z = -0.05;
    table.primitive_poses[0].orientation.w = 1.0;

    table.operation = table.ADD;
    psi.applyCollisionObjects({table});
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test2_pick_place_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface psi;
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    group.setPlanningTime(45.0);
    group.setEndEffectorLink("gripper_tcp");

    addTable(psi);
    ros::Duration(1.0).sleep();

    std::vector<double> start_joints = {
        deg2rad(65.0), deg2rad(-81.0), deg2rad(46.0),
        deg2rad(-54.0), deg2rad(-90.0), deg2rad(115.0)
    };
    group.setJointValueTarget(start_joints);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        group.execute(plan);
        ros::Duration(2.0).sleep();
    } else {
        ROS_WARN("Failed to move to start pose.");
        return 1;
    }

    tf2::Quaternion q;
    q.setRPY(M_PI, 0, -M_PI / 2);
    geometry_msgs::Pose pose;
    pose.orientation = tf2::toMsg(q);

    const double lift_z = 0.12;

    geometry_msgs::Pose pose1 = pose;
    pose1.position.x = 0.15; pose1.position.y = 0.35; pose1.position.z = 0.031;

    geometry_msgs::Pose pose2 = pose1;
    pose2.position.x = -0.1; pose2.position.y = 0.3; pose2.position.z = 0.03;

    geometry_msgs::Pose pose3 = pose2;
    pose3.position.x = 0.15; pose3.position.y = 0.295; pose3.position.z = 0.031;

    geometry_msgs::Pose pose4 = pose3;
    pose4.position.x = -0.1; pose4.position.y = 0.3; pose4.position.z = 0.057;

    geometry_msgs::Pose pose5 = pose4;
    pose5.position.x = 0.15; pose5.position.y = 0.2445; pose5.position.z = 0.031;

    geometry_msgs::Pose pose6 = pose5;
    pose6.position.x = -0.1; pose6.position.y = 0.3; pose6.position.z = 0.085;

    moveCartesian(group, {pose1});
    callGripperService("/gripper_close");

    moveCartesian(group, {pose1, lifted(pose1, lift_z)});
    moveCartesian(group, {lifted(pose1, lift_z), lifted(pose2, lift_z)});
    moveCartesian(group, {lifted(pose2, lift_z), pose2});
    callGripperService("/gripper_open");

    moveCartesian(group, {pose2, lifted(pose2, lift_z), lifted(pose3, lift_z), pose3});
    callGripperService("/gripper_close");

    moveCartesian(group, {pose3, lifted(pose3, lift_z), lifted(pose4, lift_z), pose4});
    callGripperService("/gripper_open");

    moveCartesian(group, {pose4, lifted(pose4, lift_z), lifted(pose5, lift_z), pose5});
    callGripperService("/gripper_close");

    moveCartesian(group, {pose5, lifted(pose5, lift_z), lifted(pose6, lift_z), pose6});
    callGripperService("/gripper_open");

    ros::waitForShutdown();
    return 0;
}