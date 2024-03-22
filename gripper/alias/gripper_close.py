#!/usr/bin/env python3

from RobotiqHand import RobotiqHand

HOST = "192.168.56.2"
PORT = 54321

hand = RobotiqHand()

hand.connect(HOST, PORT)
print("Connected")

hand.move(255, 255, 0)

hand.wait_move_complete()

hand.disconnect()
print("Disconnected")

# import rospy
# import actionlib
# from gripper_action_server.msg import GripperControlAction, GripperControlGoal, GripperControlResult
#
# rospy.init_node('gripper_client')
# client = actionlib.SimpleActionClient('/gripper_action_server', GripperControlAction)
# client.wait_for_server()
#
# goal = GripperControlGoal(goal="close", speed=255)
# client.send_goal(goal)
