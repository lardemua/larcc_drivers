#!/usr/bin/env python3

import rospy
import actionlib
from gripper_action_server.msg import GripperControlAction, GripperControlGoal, GripperControlResult

import time


def MyActionClient(cancel):
    client = actionlib.SimpleActionClient('/gripper_action_server', GripperControlAction)
    client.wait_for_server()

    goal = GripperControlGoal(goal="close", speed=255)
    client.send_goal(goal)
    # time.sleep(2)
    # client.cancel_goal()

    # goal = GripperControlGoal(goal="open", speed=0)
    # client.send_goal(goal)



    # client.cancel_goal()
    # result = client.wait_for_result()





    # time.sleep(2)
    # client.cancel_goal()
    # goal = GripperControlGoal(goal="open", speed=0)
    # client.send_goal(goal)
    # if cancel != 1:
    client.wait_for_result()
    return client.get_result()  # Result
    # else:
    #     time.sleep(2)
    #     client.cancel_goal()
    #     # client.cancel_all_goals()
    #     result = GripperControlResult()
    #     result.result = "Action Canceled"
    #     return result


if __name__ == '__main__':
    try:
        rospy.init_node('simple_gripper_action_client')
        cancel = 0  # 1 to cancel goal with 5 delay secs
        result = MyActionClient(cancel)
        # print("Result:" + str(result.result))
        print("Result: \n")
        print(result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion")


