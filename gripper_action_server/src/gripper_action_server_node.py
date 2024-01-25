#!/usr/bin/env python3

import rospy
from actionlib.action_server import ActionServer
from gripper_action_server.msg import GripperControlAction, GripperControlFeedback, GripperControlResult
from RobotiqHand import RobotiqHand
import actionlib_msgs.msg
import threading
import collections

# Named tuple for storing the goal handle and the corresponding processing thread
GoalHandleThread = collections.namedtuple('GoalHandleThread', 'goal_handle thread')


class GripperActionServer(ActionServer):
    """ Tutorial for how to write a ros action server in python.
            Works with multiple paralell goals, since it creates a processing thread for each newly received (and accepted) goal.
        """
    _threads = {}  # a dictionary containing a GoalHandleThread tuple for each goal processing thread

    def __init__(self, name):
        """ Initializes the actions sever

        :param name: name of the action server
        """
        self.server_name = name
        self.counter_th = 20
        self.hand = RobotiqHand()
        self.hand.connect("192.168.56.2", 54321)
        self.hand.reset()
        self.hand.activate()
        self.hand.wait_activate_complete()

        actual_pos = self.hand.get_instant_gripper_status().actual_position
        print("\n ACTUAL POSITION:")
        print(actual_pos)

        ActionServer.__init__(self, name, GripperControlAction, self.goalCallback, self.cancelCallback,
                              False)  # initialize superclass
        rospy.loginfo("ActionServer " + name + " initialized.")

        self.start()  # start the action server
        rospy.loginfo("ActionServer " + name + " started.")

    def goalCallback(self, gh):
        """ Called whenever a new goal is received

        :param gh: a handle to the goal
        """
        # Analyse requested goal and decide whether to accept it or not
        goal = gh.get_goal()
        id = gh.get_goal_id().id
        rospy.loginfo("Received request for goal " + str(id))

        # Goal acceptance criteria:
        if (goal.goal == "open" or goal.goal == "close") and (0 <= goal.speed <= 255) and (
                self.hand.get_instant_gripper_status().is_activated):
            gh.set_accepted()  # accept goal
            _current_goal = goal
            thread = threading.Thread(target=self.processGoal, args=(gh,))  # create a thread to process this goal
            self._threads[id] = (GoalHandleThread(gh, thread))  # add to tasks dictionary
            thread.start()  # initiate thread
            # rospy.logwarn("Accepted goal request. Launched a processing thread for goal id " + str(id))
        else:  # goal rejection
            # rospy.logwarn("Rejected goal request for goal id " + str(id))
            result = GripperControlResult()  # create an empty result class instance
            result.result = "Goal can only be 'open' or 'close'. Speed can only be integers between 0 and 255. Gripper should be activated."
            result.position = self.hand.get_instant_gripper_status().actual_position
            result.object_detected = self.hand.get_instant_gripper_status().object_detected

            gh.set_rejected(result=result,
                            text="Goal can only be 'open' or 'close'. \n "
                                 "Speed can only be integers between 0 and 255.  \n "
                                 "Gripper should be activated.")

    def cancelCallback(self, gh):
        """ Called when a cancel request is received.

        :param gh: a handle to the goal
        """
        goal = gh.get_goal()  # get the goal
        id = gh.get_goal_id().id  # get the goal id
        rospy.logerr("Received cancel request for goal " + str(id))
        result = GripperControlResult()  # create an empty result class instance
        result.result = "Goal canceled"
        result.position = self.hand.get_instant_gripper_status().actual_position
        result.object_detected = self.hand.get_instant_gripper_status().object_detected
        gh.set_canceled(result=result, text="Canceled.")  # cancel the goal

    def processGoal(self, gh):
        """ Processes the goal.
        Called in a separate thread(s), so that it does not interfere with the actionlib state machine

        :param gh: a handle to the goal
        :return:
        """
        # Get the goal handle and the thread using the id as dictionary key
        goal = gh.get_goal()
        id = gh.get_goal_id().id
        _, thread = self._threads[id]  # shows how to get the thread knowing the goal id

        r = rospy.Rate(100)  # 100hz
        old_pos = self.hand.get_instant_gripper_status().actual_position
        counter = 0

        while not rospy.is_shutdown():
            r.sleep()
            actual_pos = self.hand.get_instant_gripper_status().actual_position

            # Check if goal is active, if it is not active, interrupt processing
            if not gh.get_goal_status().status == actionlib_msgs.msg.GoalStatus.ACTIVE:
                rospy.logerr("Goal " + str(id) + " canceled.")
                del self._threads[id]  # remove from dictionary
                return  # the thread will terminate once the return is called

            if actual_pos == old_pos and counter == self.counter_th:
                # rospy.logwarn("Completed goal " + str(id))

                fb = GripperControlFeedback()
                fb.feedback = "Goal achieved successfuly."
                fb.position = actual_pos
                fb.object_detected = self.hand.get_instant_gripper_status().object_detected
                fb.status = 1
                gh.publish_feedback(feedback=fb)

                result = GripperControlResult()  # create an empty result class instance
                result.result = "Goal achieved successfuly."
                result.position = actual_pos
                result.object_detected = self.hand.get_instant_gripper_status().object_detected
                gh.set_succeeded(result=result)
                del self._threads[id]  # remove from dictionary
                return  # the thread will terminate once the return is called
            else:
                if goal.goal == "open":
                    self.hand.move(0, goal.speed, 255)

                    fb = GripperControlFeedback()
                    fb.feedback = "Opening Gripper..."
                    fb.position = actual_pos
                    fb.object_detected = self.hand.get_instant_gripper_status().object_detected
                    fb.status = 0
                    gh.publish_feedback(feedback=fb)

                    # rospy.loginfo("Processing goal: Opening Gripper... ")
                elif goal.goal == "close":
                    self.hand.move(255, goal.speed, 255)

                    fb = GripperControlFeedback()
                    fb.feedback = "Closing Gripper..."
                    fb.position = actual_pos
                    fb.object_detected = self.hand.get_instant_gripper_status().object_detected
                    fb.status = 0
                    gh.publish_feedback(feedback=fb)

                    # rospy.loginfo("Processing goal: Closing Gripper... ")

            if counter < self.counter_th:
                counter += 1
            else:
                old_pos = self.hand.get_instant_gripper_status().actual_position
                counter = 0


if __name__ == '__main__':
    rospy.init_node("gripper_action_server")
    gripper_action_server = GripperActionServer("gripper_action_server")
    rospy.spin()
