

import sys
import rospy
import numpy as np
from pau2motors.msg import pau
import math
import tf

def talker():
    pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    rospy.init_node('head2pau', anonymous=True)

    rate = rospy.Rate(120) # 29fps approx. measure this exactly.

    data = np.loadtxt(sys.argv[1], delimiter=' ', skiprows=2)
    angles = data[:,2:5]

    rospy.loginfo("loaded file with %d frames." % len(data))

    angle_range = 45.0


    for frame in angles:
        msg = pau()

        print frame

        roll = np.deg2rad(frame[0])
        pitch = np.deg2rad(-frame[2])
        yaw = np.deg2rad(frame[1])
        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        print quaternion
        msg.m_headRotation.x = quaternion[0]
        msg.m_headRotation.y = quaternion[1]
        msg.m_headRotation.z = quaternion[2]
        msg.m_headRotation.w = quaternion[3]

        msg.m_headTranslation.x = 0
        msg.m_headTranslation.y = 0
        msg.m_headTranslation.z = 0

        msg.m_neckRotation.x = 0
        msg.m_neckRotation.y = 0
        msg.m_neckRotation.z = 0
        msg.m_neckRotation.w = 0

        msg.m_eyeGazeLeftPitch = 0
        msg.m_eyeGazeLeftYaw = 0

        msg.m_eyeGazeRightPitch = 0
        msg.m_eyeGazeRightYaw = 0

        print("sending frame...")

        msg.m_shapekeys = ['']


        pub.publish(msg)

        rate.sleep()

        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    if (len(sys.argv)!=2):
        rospy.loginfo('Not enough or too many arguments, specify input CSV file.')

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
