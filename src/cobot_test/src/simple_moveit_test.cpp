#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_moveit_test");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    move_group.setPlanningTime(10.0);
    move_group.setEndEffectorLink("tool0");  // Use "tool0" or your actual RG2 tip link

    // Get current robot state as planning start point
    move_group.setStartStateToCurrentState();

    // Define a simple pose
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.15;
    target_pose.position.y = -0.1;
    target_pose.position.z = 0.25;

    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, 0);  // Upright neutral pose
    target_pose.orientation = tf2::toMsg(orientation);

    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("✅ Plan to pick position successful. Executing...");
        move_group.execute(plan);
    }
    else
    {
        ROS_WARN("❌ Failed to plan to pick pose.");
        return 1;
    }

    ros::Duration(2.0).sleep();

    // Now move to a different place (place position)
    target_pose.position.y = 0.1;
    move_group.setPoseTarget(target_pose);

    if (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("✅ Plan to place position successful. Executing...");
        move_group.execute(plan);
    }
    else
    {
        ROS_WARN("❌ Failed to plan to place pose.");
    }

    ros::shutdown();
    return 0;
}
