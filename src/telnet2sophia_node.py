#!/usr/bin/env python
#
# Robot PAU to csv output
# Copyright (C) 2016 Hanson Robotics
# Author: Ralf Mayet
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
# 02110-1301  USA
#

#
# clip 0
# scale to max 1
#

import sys
import rospy
import numpy as np
from pau2motors.msg import pau
import telnetlib

def talker():
    pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    rospy.init_node('telnet2pau', anonymous=True)

    HOST = sys.argv[1]
    PORT = 8888

    tn = telnetlib.Telnet(HOST, PORT)

    rospy.loginfo("Opened connection...")

    angle_range = 45.0

    tn.read_until("\n")

    while 1:

        msg = pau()

        msg.m_headRotation.x = 0
        msg.m_headRotation.y = 0
        msg.m_headRotation.z = 0
        msg.m_headRotation.w = 0

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


        ## Read the frame
        frame_str = tn.read_until("\n").strip("\n")

        print(frame_str)

        frame = np.array(frame_str.split(","))
        frame = frame.astype(np.float)

        frame *= 0.01

    	frame = np.power(frame, 2.0)
    	frame = np.minimum(1.0, frame)

        ## Delete superfluous frames:
        cut_frame = np.delete(frame, np.s_[13:27], axis=0)

        msg.m_coeffs = cut_frame


        msg.m_shapekeys = ['brow_center_DN', 'brow_center_UP', 'brow_inner_DN.L', 'brow_inner_UP.L', 'brow_inner_DN.R', 'brow_inner_UP.R', 'brow_outer_DN.L', 'brow_outer_UP.L', 'brow_outer_DN.R', 'brow_outer_up.R', 'wince.L', 'wince.R', 'lip-JAW.DN', 'eye-blink.LO.L', 'eye-flare.LO.L', 'eye-blink.LO.R', 'eye-flare.LO.R', 'eye-blink.UP.L', 'eye-flare.UP.L', 'eye-blink.UP.R', 'eye-flare.UP.R', 'lips-wide.L', 'lips-narrow.L', 'lips-wide.R', 'lips-narrow.R', 'lips-frown.L', 'lips-frown.R', 'lip-DN.C.DN', 'lip-DN.C.UP', 'lip-DN.L.DN', 'lip-DN.L.UP', 'lip-DN.R.DN', 'lip-DN.R.UP', 'lips-smile.L', 'lips-smile.R', 'lip-UP.C.DN', 'lip-UP.C.UP', 'lip-UP.L.DN', 'lip-UP.L.UP', 'lip-UP.R.DN', 'lip-UP.R.UP', 'sneer.L', 'sneer.R']

        assert(len(msg.m_shapekeys) == len(cut_frame))

        print("sending frame...")
        pub.publish(msg)

        if rospy.is_shutdown():
            break

if __name__ == '__main__':
    if (len(sys.argv)!=2):
        rospy.loginfo('Not enough or too many arguments, specify input CSV file.')

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
