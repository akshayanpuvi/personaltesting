#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gripper_visual_test_node");
    ros::NodeHandle nh;
    ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

    ros::Rate rate(10); // 10 Hz loop

    sensor_msgs::JointState msg;
    msg.name.push_back("finger_width");
    msg.position.push_back(0.0);  // fully closed

    ROS_INFO("[Gripper Visual Test] Publishing constant closed gripper...");

    while (ros::ok())
    {
        msg.header.stamp = ros::Time::now();
        joint_pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
