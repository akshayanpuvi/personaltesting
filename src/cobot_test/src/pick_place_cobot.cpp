#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// tau = 1 rotation in radians
const double tau = 2 * M_PI;

// Helper function to convert degrees to radians
inline double deg2rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

void controlGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, const std::string& action)
{
    gripper_group.setNamedTarget(action);  // 'open' or 'close'
    gripper_group.move();
    ros::Duration(1.0).sleep();  // Let it complete
}

void moveToPose(moveit::planning_interface::MoveGroupInterface& move_group,
                moveit::planning_interface::MoveGroupInterface& gripper_group)
{

    // Step 1: Move to initial joint configuration for better pick up posture of the UR3e

    std::vector<double> start_joint_angles = {
        deg2rad(65.0),    // shoulder_pan_joint
        deg2rad(-81.0),   // shoulder_lift_joint
        deg2rad(46.0),    // elbow_joint
        deg2rad(-54.0),   // wrist_1_joint
        deg2rad(-90.0),   // wrist_2_joint
        deg2rad(115.0)    // wrist_3_joint
    };

    // Get current joint values to maintain correct vector size/order
    std::vector<double> joint_group_positions = move_group.getCurrentJointValues();

    // Make sure your vector sizes match
    if (joint_group_positions.size() != start_joint_angles.size())
    {
        ROS_ERROR("Joint count mismatch! Expected %lu, got %lu.", joint_group_positions.size(), start_joint_angles.size());
        return;
    }

    joint_group_positions = start_joint_angles;
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan start_plan;
    if (move_group.plan(start_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        ROS_INFO("Moving to start joint configuration...");
        move_group.execute(start_plan);
        ros::Duration(2.0).sleep();
    }
    else
    {
        ROS_WARN("Failed to plan start joint configuration.");
        return;
    }

    // Step 2: Continue with Cartesian poses 

    tf2::Quaternion orientation;
    orientation.setRPY(M_PI, 0, -M_PI / 2);
    geometry_msgs::PoseStamped current = move_group.getCurrentPose();

    geometry_msgs::Pose start_pose = current.pose;
    start_pose.orientation = tf2::toMsg(orientation);


    // 3 Type Parcels (Standard, Express, Fragile) into seperate zones

    // Define poses as in your original code

    //START - EXPRESS
    geometry_msgs::Pose pose1 = start_pose;
    pose1.position.x = 0.15;
    pose1.position.y = 0.35;
    pose1.position.z = 0.031;
    pose1.orientation = tf2::toMsg(orientation);

    //END - EXPRESS AREA
    geometry_msgs::Pose pose2 = pose1;
    pose2.position.x = -0.1;
    pose2.position.y = 0.395;

    //START - FRAGILE
    geometry_msgs::Pose pose3 = pose2;
    pose3.position.x = 0.15;
    pose3.position.y = 0.295;

    //END - FRAGILE AREA
    geometry_msgs::Pose pose4 = pose3;
    pose4.position.x = -0.2;
    pose4.position.y = 0.2445;

    //START - STANDARD
    geometry_msgs::Pose pose5 = pose4;
    pose5.position.x = 0.15;
    pose5.position.y = 0.2445;

    //END - STANDARD AREA
    geometry_msgs::Pose pose6 = pose5;
    pose6.position.x = -0.1;
    pose6.position.y = 0.2445;


    const double lift_z = 0.12; // Added lift to avoid any collision 

    auto lifted = [](const geometry_msgs::Pose& pose, double dz) {
        geometry_msgs::Pose p = pose;
        p.position.z += dz;
        return p;
    };

    auto move_cartesian = [&](const std::vector<geometry_msgs::Pose>& waypoints) {
        moveit_msgs::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        const double jump_threshold = 0.0;

        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        if (fraction > 0.95)
        {
            ROS_INFO("Path planned (%.2f%%). Executing...", fraction * 100.0);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            move_group.execute(plan);
        }
        else
        {
            ROS_WARN("Path planning incomplete (%.2f%%). Skipping...", fraction * 100.0);
        }

        ros::Duration(2.0).sleep();
    };


    // Movement of UR3e to designated parcels/area --------------------------------------------------------------

    // POSE 1

    move_cartesian({
        start_pose,
        lifted(start_pose, lift_z),
        lifted(pose1, lift_z),
        pose1, 
    });
    
    controlGripper(gripper_group, "open");


    // POSE 2
 
    // Move to pose1 (if not already there)
    move_cartesian({
        start_pose,    
        pose1
    });

    ros::Duration(2.0).sleep();   // Optional pause before gripper close

    controlGripper(gripper_group, "close");  // Close gripper here

    ros::Duration(1.0).sleep();   // Wait for gripper action to complete

    // Lift up from POSE1 AFTER gripper closes
    move_cartesian({
        pose1,
        lifted(pose1, lift_z)
    });

    // Move horizontally (lifted POSE1 to lifted POSE2)
    move_cartesian({
        lifted(pose1, lift_z),
        lifted(pose2, lift_z)
    });

    // Lower down to POSE2
    move_cartesian({
        lifted(pose2, lift_z),
        pose2
    });



    // POSE 3

    controlGripper(gripper_group, "open");

    move_cartesian({
        pose2,
        lifted(pose2, lift_z),
        lifted(pose3, lift_z),
        pose3,
    });
    controlGripper(gripper_group, "close");

    // POSE 4

    move_cartesian({
        pose3,
        lifted(pose3, lift_z),
        lifted(pose4, lift_z),
        pose4,
    });
    controlGripper(gripper_group, "open");

    // POSE 5

    move_cartesian({
        pose4,
        lifted(pose4, lift_z),
        lifted(pose5, lift_z),
        pose5,
    });
    controlGripper(gripper_group, "close");

    // POSE 6

    move_cartesian({
        pose5,
        lifted(pose5, lift_z),
        lifted(pose6, lift_z),
        pose6,
    });
    controlGripper(gripper_group, "open");
}



void addCollisionObject(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the table to represnt the UR3e table 
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "base_link";

    // Define overall size of the table in simulation to act as a collision object
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8; //x
    collision_objects[0].primitives[0].dimensions[1] = 0.7; //y
    collision_objects[0].primitives[0].dimensions[2] = 0.1; //z

    // Pose of the table in simulation
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = 0.23;
    collision_objects[0].primitive_poses[0].position.z = -0.05;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;

    // Adds the table to the scene
    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cobot_pick_and_place");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep(); // Give time for system to initialize

    // Planning scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    addCollisionObject(planning_scene_interface);

    ros::WallDuration(1.0).sleep(); // Wait for object to register

    // Move group
    moveit::planning_interface::MoveGroupInterface group("manipulator");
    moveit::planning_interface::MoveGroupInterface gripper_group("onrobot");
    group.setPlanningTime(45.0);
    group.setEndEffectorLink("gripper_tcp");

    ROS_INFO("End effector link: %s", group.getEndEffectorLink().c_str());

    moveToPose(group, gripper_group);

    ros::waitForShutdown();
    return 0;
}
