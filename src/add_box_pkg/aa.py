#!/usr/bin/env python

import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def add_collision_box():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_box_node_py', anonymous=True)

    planning_scene_interface = moveit_commander.PlanningSceneInterface()
    rospy.sleep(2.0)  # Let the interface initialize

    # Define collision object
    box = CollisionObject()
    box.id = "separator"
    box.header.frame_id = "base_link"
    box.operation = CollisionObject.ADD

    # Define box shape
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.02, 0.2, 0.1]  # x, y, z

    # Define pose
    pose = Pose()
    pose.position.x = 0.0
    pose.position.y = 0.35
    pose.position.z = 0.05  # Half of height above ground
    pose.orientation.w = 1.0

    box.primitives = [primitive]
    box.primitive_poses = [pose]

    # Apply object
    planning_scene_interface.apply_collision_objects([box])
    rospy.loginfo("Added 'separator' collision object to the planning scene.")

    rospy.sleep(3.0)  # Wait to visualize in RViz

if __name__ == '__main__':
    import sys
    add_collision_box()







# import moveit_commander
# import sys
# import rospy

# rospy.init_node('get_ee_link_name', anonymous=True)
# moveit_commander.roscpp_initialize(sys.argv)

# group = moveit_commander.MoveGroupCommander("manipulator")
# ee_link = group.get_end_effector_link()
# print("End effector link:", ee_link)

# moveit_commander.roscpp_shutdown()

















# #!/usr/bin/env python3

# import sys
# import rospy
# import moveit_commander
# import math

# def move_to_joint_positions():
#     moveit_commander.roscpp_initialize(sys.argv)
#     rospy.init_node('move_to_joint_positions_node', anonymous=True)

#     # Instantiate RobotCommander and MoveGroupCommander
#     robot = moveit_commander.RobotCommander()
#     scene = moveit_commander.PlanningSceneInterface()
#     group = moveit_commander.MoveGroupCommander("manipulator")

#     # Target joint positions in radians
#     joint_goal = [0, -math.pi/2, 0, -math.pi/2, 0, 0]

#     # Set the joint target
#     group.go(joint_goal, wait=True)

#     # Ensure that there is no residual movement
#     group.stop()

#     current_joints = group.get_current_joint_values()
#     rospy.loginfo("Current joint positions: {}".format(current_joints))

#     moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     try:
#         move_to_joint_positions()
#     except rospy.ROSInterruptException:
#         pass
