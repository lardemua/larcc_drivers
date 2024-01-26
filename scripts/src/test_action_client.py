#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionGoal, MoveGroupActionFeedback, MoveGroupActionResult, GenericTrajectory
from moveit_msgs.msg import MoveGroupGoal, BoundingVolume
from UR10eArm import UR10eArm
import tf
from geometry_msgs.msg import Vector3, Pose, Quaternion
from shape_msgs.msg import SolidPrimitive
from gripper_action_server.msg import GripperControlAction, GripperControlGoal, GripperControlResult, \
    GripperControlActionFeedback

joint_values_1 = [1.1278675238238733, -0.9531238240054627, -0.08442861238588506, -2.4318977795042933, -1.5508225599872034, 0.026221752166748047]
joint_values_2 = [2.187239472066061, -2.0731855831541957, -0.05339032808412725, -2.148046155969137, -1.473081413899557, -0.2870219389544886]


def send_goal_joint_states(client, jv):
    goal = MoveGroupGoal()
    constraints = Constraints()
    joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint",
                   "wrist_3_joint"]
    for joint_name, joint_value in zip(joint_names, jv):
        constraint = JointConstraint()
        constraint.joint_name = joint_name
        constraint.position = joint_value
        constraint.tolerance_above = 0.0001
        constraint.tolerance_below = 0.0001
        constraint.weight = 1.0
        constraints.joint_constraints.append(constraint)

    goal.request.goal_constraints = [constraints]
    goal.request.group_name = 'manipulator'
    goal.request.num_planning_attempts = 10
    goal.request.allowed_planning_time = 10
    goal.request.max_velocity_scaling_factor = 1
    goal.request.max_acceleration_scaling_factor = 1
    goal.request.workspace_parameters.header.stamp = rospy.Time.now()
    goal.request.workspace_parameters.header.frame_id = 'world'

    # Sends the goal to the action server.
    client.send_goal(goal)

    current_status = client.get_state()
    print("Current action status: {}".format(current_status))


# def send_goal_tool_pose(client, position, orientation):
#     goal = MoveGroupGoal()
#     constraints = Constraints()
#
#     position_constraint = PositionConstraint()
#     position_constraint.link_name = "base_link"
#     # listener = tf.TransformListener()
#     # listener.waitForTransform('/base', '/tool0_controller', rospy.Time(), rospy.Duration(4.0))
#     # (trans, rot) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
#     offset = Vector3()
#     # offset.x = position[0] - trans[0]
#     # offset.y = position[1] - trans[1]
#     # offset.z = position[2] - trans[2]
#     #
#     offset.x = position[0]
#     offset.y = position[1]
#     offset.z = position[2]
#
#     position_constraint.target_point_offset = offset
#     position_constraint.weight = 1.0
#
#     quaternions = Quaternion()
#     quaternions.x = orientation[0]
#     quaternions.y = orientation[1]
#     quaternions.z = orientation[2]
#     quaternions.w = orientation[3]
#
#     orientation_constraint = OrientationConstraint()
#     orientation_constraint.orientation = quaternions
#     orientation_constraint.weight = 1.0
#
#     constraints.position_constraints.append(position_constraint)
#     constraints.orientation_constraints.append(orientation_constraint)
#
#     goal.request.goal_constraints = [constraints]
#     goal.request.group_name = 'manipulator'
#     goal.request.num_planning_attempts = 10
#     goal.request.allowed_planning_time = 10
#     goal.request.max_velocity_scaling_factor = 1
#     goal.request.max_acceleration_scaling_factor = 1
#     goal.request.workspace_parameters.header.stamp = rospy.Time.now()
#     goal.request.workspace_parameters.header.frame_id = 'base_link'
#
#     # Sends the goal to the action server.
#     client.send_goal(goal)
#
#     current_status = client.get_state()
#     print("Current action status: {}".format(current_status))


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_action_client')

        arm = UR10eArm()
        # print("\nmove_group.get_end_effector_link")
        # print(arm.move_group.get_end_effector_link())
        arm.go_to_pose_goal(0.2809661907309665, 0.4002262425682399, 1.1649774428316204, -0.12557957928354258,
                            0.9640954500733134, -0.23382996598184058, 0.008560340170532865, 1, 1)

        # listener = tf.TransformListener()
        # listener.waitForTransform('/world', '/end_effector', rospy.Time(), rospy.Duration(4.0))
        # (trans, rot) = listener.lookupTransform('/world', '/end_effector', rospy.Time(0))
        # print("\ntrans")
        # print(trans)
        # print("\nrot")
        # print(rot)


        # cancel = 0  # 1 to cancel goal with 5 delay secs


        # gripper_client = actionlib.SimpleActionClient('/gripper_action_server', GripperControlAction)


        # gripper_client.wait_for_server()

        # order = "open"
        # order = "close"
        # goal = GripperControlGoal(goal=order, speed=50)
        # gripper_client.send_goal(goal)

        # arm_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        # arm_client.wait_for_server()
        # send_goal_joint_states(arm_client, joint_values_1)



        # position = [-0.5506018128638124, -0.18126909584712633, 0.38537764173340205]
        # orientation = [-0.5933010498866534, -0.770049458736081, 0.1718641863720918, 0.15962580225741207]
        # send_goal_tool_pose(arm_client, position, orientation)

        # arm_client.cancel_goal()

        # arm_client.wait_for_result()

        # print("Current action status: {}".format(arm_client.get_state()))
        # print("Current action result: {}".format(arm_client.get_result()))
        # gripper_client.cancel_goal()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
