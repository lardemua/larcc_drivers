#!/usr/bin/env python3

import actionlib
import json
import rospy
import sys

from arm.srv import MoveArmToJointsState, MoveArmToJointsStateRequest
from gripper_action_server.msg import GripperControlAction, GripperControlGoal


rospy.init_node("arm_gripper_movement", anonymous=True)

try:
    f = open(rospy.get_param(rospy.search_param('joints_poses')), "r")
    positions = json.load(f)
    positions = positions["positions"]
    positions = list(positions.items())
    positions = [["open"], ["close"]] + positions
    f.close()

except:
    rospy.logerr("Invalid positions file! Closing...")
    sys.exit(0)

# set up arm controller service proxy
rospy.wait_for_service('move_arm_to_joints_state')
move_arm_to_joints_state_proxy = rospy.ServiceProxy('move_arm_to_joints_state', MoveArmToJointsState)

# set up gripper action client
gripper_control_client = actionlib.SimpleActionClient('/gripper_action_server', GripperControlAction)
gripper_control_client.wait_for_server()

while True:
    i = 0

    for pos in positions:
        print(f'[{i}]:' + pos[0])
        i += 1

    idx = int(input("Select idx: "))

    if idx == 0:
        goal = GripperControlGoal(goal="open", speed=255)
        gripper_control_client.send_goal(goal)
        gripper_control_client.wait_for_result()
        print(gripper_control_client.get_result())

    elif idx == 1:
        goal = GripperControlGoal(goal="close", speed=255)
        gripper_control_client.send_goal(goal)
        gripper_control_client.wait_for_result()
        print(gripper_control_client.get_result())

    else:
        pos = positions[idx][1]

        req = MoveArmToJointsStateRequest(goal=pos, velocity=0.2, acceleration=0.2, wait=False)

        try:
            resp = move_arm_to_joints_state_proxy(req)
            print(resp)

        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            sys.exit(0)
