#! /usr/bin/python3

import numpy as np
import rospy
import rtde_control
import rtde_receive
import tf

from scipy.spatial.transform import Rotation as R


def quaternion_to_rotvec(quaternion):
    """
    Converts a quaternion to a rotation vector.
    
    Parameters:
    quaternion (array-like): A quaternion [w, x, y, z]
    
    Returns:
    numpy.ndarray: A rotation vector [rx, ry, rz]
    """
    w, x, y, z = quaternion
    angle = 2 * np.arccos(w)
    s = np.sqrt(1 - w*w)  # sin(angle/2)
    
    if s < 1e-8:  # To avoid division by zero
        return np.array([0, 0, 0])
    
    return angle * np.array([x, y, z]) / s


def rotvec_to_quaternion(rotvec):
    """
    Converts a rotation vector to a quaternion.
    
    Parameters:
    rotvec (array-like): A rotation vector [rx, ry, rz]
    
    Returns:
    numpy.ndarray: A quaternion [w, x, y, z]
    """
    theta = np.linalg.norm(rotvec)
    
    if theta < 1e-8:
        return np.array([1, 0, 0, 0])
    
    axis = rotvec / theta
    w = np.cos(theta / 2)
    xyz = axis * np.sin(theta / 2)
    
    return np.hstack(([w], xyz))


def calculate_angular_velocity(current_orientation, desired_orientation, gain=1.0):
    # Convert orientations to scipy Rotation objects
    current_rot = R.from_quat(current_orientation)
    desired_rot = R.from_quat(desired_orientation)

    # Calculate the rotation needed to go from current to desired orientation
    delta_rot = desired_rot * current_rot.inv()
    rotvec = delta_rot.as_rotvec()  # Convert to rotation vector

    # Scale by gain to control the speed of adjustment
    angular_velocity = gain * rotvec
    return angular_velocity


class RTVelController:
    """UR10eArm movement controller using velocity control to track a goal pose."""

    def __init__(self):
        self.rtde_c = rtde_control.RTDEControlInterface("192.168.56.2")
        self.rtde_r = rtde_receive.RTDEReceiveInterface("192.168.56.2")

        self.tf_broadcaster = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
    
    def loop(self):
        rate  = rospy.Rate(100)

        while not rospy.is_shutdown():
            if not self.listener.frameExists("/goal") or not self.listener.frameExists("/base"):
                rate.sleep()
                continue

            self.listener.waitForTransform('/base', '/goal', rospy.Time(), rospy.Duration(1))
            (trans, rot) = self.listener.lookupTransform('/base', '/goal', rospy.Time())

            current = np.array(self.rtde_r.getActualTCPPose())
            temp = rotvec_to_quaternion(current[3:])
            current = np.hstack((current[0:3], temp[1:], [temp[0]])) # [x, y, z, qx, qy, qz, qw]

            goal = np.hstack((trans, rot)) # [x, y, z, qx, qy, qz, qw]

            diff = np.hstack((goal[0:3] - current[0:3], calculate_angular_velocity(current[3:], goal[3:])))

            if any(abs(diff) > 0.01):
                if np.linalg.norm(diff[0:3]) > 0.05:
                    diff[0:3] = diff[0:3]/np.linalg.norm(diff[0:3])*0.4
                    diff[3:6] = diff[3:6]/np.linalg.norm(diff[3:6])*0.1
                
                if np.linalg.norm(diff[0:3]) < 0.05:
                    diff[0:3] = diff[0:3]/np.linalg.norm(diff[0:3])*0.1
                    diff[3:6] = diff[3:6]/np.linalg.norm(diff[3:6])*0.1

                self.rtde_c.jogStart(diff, acc=1)
            
            else:
                self.rtde_c.jogStop()
            
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('ur10e_arm', anonymous=True)

    arm = RTVelController()

    arm.loop()

    arm.rtde_c.jogStop()

    rospy.spin()
