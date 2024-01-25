#! /usr/bin/python3

from UR10eArm import UR10eArm
import rospy

if __name__ == '__main__':
    rospy.init_node("get_current_joint_values", anonymous=True)
    # --------------------------------------------------------------------
    # -------------------------initialization-----------------------------
    # --------------------------------------------------------------------
    tutorial = UR10eArm()
    move_group = tutorial.move_group

    current_pose = move_group.get_current_pose().pose
    print("current_pose")
    print(current_pose)

    current_joints = move_group.get_current_joint_values()
    print("current_joints")

    # Leave this way!!!! truly important. That's because the action goal is sent in this order:
    # ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    print([current_joints[2], current_joints[1], current_joints[0], current_joints[3], current_joints[4], current_joints[5]])
