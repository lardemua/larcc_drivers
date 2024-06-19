#! /usr/bin/python3

import moveit_commander
import rospy
import sys

from arm.srv import MoveArmToJointsState, MoveArmToJointsStateResponse
from arm.srv import MoveArmToPoseGoal, MoveArmToPoseGoalResponse
from arm.srv import StopArm, StopArmResponse


class UR10eArm(object):
    """UR10eArm movement with ROS services"""

    def __init__(self):
        super(UR10eArm, self).__init__()

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        # rospy.init_node("move_group_python_interface", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        self.scene = moveit_commander.PlanningSceneInterface()

        ## This interface can be used to plan and execute motions:
        group_name = "manipulator"
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Initialize services
        move_arm_to_pose_goal_service = rospy.Service("move_arm_to_pose_goal", MoveArmToPoseGoal, self.move_arm_to_pose_goal)
        move_arm_to_joints_state_service = rospy.Service("move_arm_to_joints_state", MoveArmToJointsState, self.move_arm_to_joints_state)
        stop_arm_service = rospy.Service("stop_arm", StopArm, self.stop_arm)

    def move_arm_to_joints_state(self, req):
        """
        Go to a certain pose goal described by the joints state
        """
        assert len(req.goal) == 6 and 0 < req.velocity <= 1 and 0 < req.acceleration <= 1

        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        goal = self.move_group.get_current_joint_values()
        goal[0] = req.goal[0]
        goal[1] = req.goal[1]
        goal[2] = req.goal[2]
        goal[3] = req.goal[3]
        goal[4] = req.goal[4]
        goal[5] = req.goal[5]

        # speed configuration
        self.move_group.set_max_velocity_scaling_factor(req.velocity)
        self.move_group.set_max_acceleration_scaling_factor(req.acceleration)

        # Now, we call the planner to compute the plan and execute it.
        self.move_group.set_joint_value_target(goal)
        success, plan, planning_time, error_code = self.move_group.plan()

        if success:
            # filter out the first points of the trajectory to avoid the robot to move to the initial position
            plan.joint_trajectory.points = [p for p in plan.joint_trajectory.points if p.time_from_start.to_sec() > 1.0]

            # execute the trajectory
            success = self.move_group.execute(plan, wait=False)

            if success:
                return MoveArmToJointsStateResponse('Arm is moving to the requested pose goal.')

            else:
                return MoveArmToJointsStateResponse('Failed to execute the planned trajectory.')
        
        else:
            return MoveArmToJointsStateResponse('Failed to plan the trajectory. Error Code: ' + str(error_code))

    def move_arm_to_pose_goal(self, req):
        """
        Go to a certain pose goal described by a quaternion tf from 'world' to the end-effector 'tool0'
        """
        assert 0 < req.velocity <= 1 and 0 < req.acceleration <= 1

        # speed configuration
        self.move_group.set_max_velocity_scaling_factor(req.velocity)
        self.move_group.set_max_acceleration_scaling_factor(req.acceleration)

        # Now, we call the planner to compute the plan and execute it.
        self.move_group.set_pose_target(req.goal)
        success, plan, planning_time, error_code = self.move_group.plan()

        # It is always good to clear your targets after planning with poses.
        self.move_group.clear_pose_targets()

        # If the plan was successful, execute it
        if success:
            # filter out the first points of the trajectory to avoid the robot to move to the initial position
            plan.joint_trajectory.points = [p for p in plan.joint_trajectory.points if p.time_from_start.to_sec() > 1.0]

            # execute the trajectory
            success = self.move_group.execute(plan, wait=False)

            if success:
                return MoveArmToPoseGoalResponse('Arm is moving to the requested pose goal.')

            else:
                return MoveArmToPoseGoalResponse('Failed to execute the planned trajectory.')
        
        else:
            return MoveArmToPoseGoalResponse('Failed to plan the trajectory. Error Code: ' + str(error_code))

    def stop_arm(self, req):
        # Calling `stop()` ensures that the robot stops and there is no residual movement
        self.move_group.stop()

        return StopArmResponse('Arm is stopped.')


if __name__ == '__main__':
    rospy.init_node('ur10e_arm', anonymous=True)

    arm = UR10eArm()

    rospy.spin()
