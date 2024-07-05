#!/usr/bin/env python
import socket
import rospy
import re
import tf
import numpy as np

HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 8080        # The port used by the server


def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def talker():
    rospy.init_node('wood_block_client')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10) # 10hz
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        while not rospy.is_shutdown():
            s.sendall(b'Hello, world')
            data = s.recv(1024)
            print("data: ", data)
            # print('Received', repr(data))
            # print(data.decode())
            x = re.findall(r"[-+]?\d*\.\d+|\d+", data.decode())
            trans = (float(x[0]), float(x[1]), float(x[2]))
            quat = (float(x[3]), float(x[4]), float(x[5]), float(x[6]))
            quat2 = (0.7071068, 0, 0, +0.7071068)
            final_quat = quaternion_multiply(quat2, quat)
            print('trans: ', trans)
            print('quat: ', final_quat)
            br.sendTransform(trans,
                             final_quat,
                             rospy.Time.now(),
                             "wood_block",
                             "camera_right_rgb_optical_frame")
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass