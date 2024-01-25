#!/usr/bin/env python3

import rospy
import actionlib
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.msg import MoveGroupAction, MoveGroupActionGoal, MoveGroupActionFeedback, MoveGroupActionResult, GenericTrajectory
from moveit_msgs.msg import MoveGroupGoal
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


if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('test_action_client')
        # cancel = 0  # 1 to cancel goal with 5 delay secs

        arm_client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        # gripper_client = actionlib.SimpleActionClient('/gripper_action_server', GripperControlAction)

        arm_client.wait_for_server()
        # gripper_client.wait_for_server()

        # order = "open"
        # order = "close"
        # goal = GripperControlGoal(goal=order, speed=255)
        # gripper_client.send_goal(goal)

        send_goal_joint_states(arm_client, joint_values_2)

        # arm_client.cancel_goal()

        arm_client.wait_for_result()

        print("Current action status: {}".format(arm_client.get_state()))
        # gripper_client.cancel_goal()

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
