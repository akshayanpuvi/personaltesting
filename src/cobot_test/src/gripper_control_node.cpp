/*

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>

ros::Publisher joint_pub;

bool openGripper(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    sensor_msgs::JointState msg;
    msg.name.push_back("finger_width");
    msg.position.push_back(0.01);  // Open position
    joint_pub.publish(msg);
    res.success = true;
    res.message = "Gripper opened (simulated).";
    ROS_INFO("Gripper opened.");
    return true;
}

bool closeGripper(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    sensor_msgs::JointState msg;
    msg.name.push_back("finger_width");
    msg.position.push_back(0.0);  // Closed position
    joint_pub.publish(msg);
    res.success = true;
    res.message = "Gripper closed (simulated).";
    ROS_INFO("Gripper closed.");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_control_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::ServiceServer open_srv = nh.advertiseService("/gripper_open", openGripper);
    ros::ServiceServer close_srv = nh.advertiseService("/gripper_close", closeGripper);

    ROS_INFO("[Gripper Node] Ready: /gripper_open and /gripper_close");

    ros::waitForShutdown();
    return 0;
}

*/

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>

ros::Publisher joint_pub;
double current_width = 0.11;  // Default to open (fully)

bool openGripper(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    current_width = 0.11;
    res.success = true;
    res.message = "Gripper opened (simulated).";
    ROS_INFO("Gripper response: %s", res.message.c_str());
    return true;
}

bool closeGripper(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    current_width = 0.0;
    res.success = true;
    res.message = "Gripper closed (simulated).";
    ROS_INFO("Gripper response: %s", res.message.c_str());
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_control_node");
    ros::NodeHandle nh;

    joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::ServiceServer open_srv = nh.advertiseService("/gripper_open", openGripper);
    ros::ServiceServer close_srv = nh.advertiseService("/gripper_close", closeGripper);

    ROS_INFO("[Gripper Node] Ready: /gripper_open and /gripper_close");

    ros::Rate rate(10);
    while (ros::ok())
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name = {"finger_width"};
        msg.position = {current_width};
        joint_pub.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
