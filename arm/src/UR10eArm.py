#! /usr/bin/python3

import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import rospy
import sys

from arm.srv import MoveArmToJointsState, MoveArmToJointsStateResponse
from arm.srv import MoveArmToPoseGoal, MoveArmToPoseGoalResponse
from arm.srv import StopArm, StopArmResponse
from math import dist, fabs, cos
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class UR10eArm(object):
    """UR10eArm movement with ROS services"""

    def __init__(self):
        super(UR10eArm, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)


        # rospy.init_node("move_group_python_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        # print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        # print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # flag indicating the robot started stopping
        self.stopping = False

        # initialize services
        move_arm_to_pose_goal_service = rospy.Service("move_arm_to_pose_goal", MoveArmToPoseGoal, self.move_arm_to_pose_goal)
        move_arm_to_joints_state_service = rospy.Service("move_arm_to_joints_state", MoveArmToJointsState, self.move_arm_to_joints_state)
        stop_arm_service = rospy.Service("stop_arm", StopArm, self.stop_arm)

    # def go_to_joint_state(self, joint1, joint2, joint3, joint4, joint5, joint6, vel, a):
    def go_to_joint_state(self, req):
        """
        Go to a certain pose goal described by the joints state
        """
        assert len(req.goal) == 6 and 0 < req.velocity <= 1 and 0 < req.acceleration <= 1

        self.stopping = False

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = req.goal[0]
        joint_goal[1] = req.goal[1]
        joint_goal[2] = req.goal[2]
        joint_goal[3] = req.goal[3]
        joint_goal[4] = req.goal[4]
        joint_goal[5] = req.goal[5]

        # speed configuration
        move_group.set_max_velocity_scaling_factor(req.velocity)
        move_group.set_max_acceleration_scaling_factor(req.acceleration)

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        success = False
        while not success and not self.stopping:
            success = move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # # For testing:
        current_joints = move_group.get_current_joint_values()

        if all_close(joint_goal, current_joints, 0.01):
            return MoveArmToJointsStateResponse('Arm is now at requested joints state goal.')
        else:
            return MoveArmToJointsStateResponse('Arm is not at requested joints state goal.')

    def go_to_pose_goal(self, req):
        """
        Go to a certain pose goal described by a quaternion tf from 'world' to the end-effector 'tool0'
        """
        assert len(req.translation) == 3 and len(
            req.quaternions) == 4 and 0 < req.velocity <= 1 and 0 < req.acceleration <= 1
        
        self.stopping = False

        move_group = self.move_group

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the end-effector:
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = req.quaternions[3]
        pose_goal.orientation.z = req.quaternions[2]
        pose_goal.orientation.y = req.quaternions[1]
        pose_goal.orientation.x = req.quaternions[0]

        pose_goal.position.x = req.translation[0]
        pose_goal.position.y = req.translation[1]
        pose_goal.position.z = req.translation[2]

        # speed configuration
        move_group.set_max_velocity_scaling_factor(req.velocity)
        move_group.set_max_acceleration_scaling_factor(req.acceleration)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = False
        while not success and not self.stopping:
            success = move_group.go(pose_goal, wait=True)

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose

        if all_close(pose_goal, current_pose, 0.01):
            return MoveArmToPoseGoalResponse('Arm is now at requested pose goal.')
        else:
            return MoveArmToPoseGoalResponse('Arm is not at requested pose goal.')

    def stop_arm(self, req):
        # robot started stopping
        self.stopping = True

        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()

        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        return StopArmResponse('Arm is stopped.')

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL


if __name__ == '__main__':
    rospy.init_node('ur10e_arm', anonymous=True)

    arm = UR10eArm()

    rospy.spin()
