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

def talker():
    pub = rospy.Publisher('/blender_api/faceshift_values', pau, queue_size=10)
    rospy.init_node('csv2pau', anonymous=True)

    rate = rospy.Rate(48) # 29fps approx. measure this exactly.

    data = np.loadtxt(sys.argv[1], delimiter=',')

    rospy.loginfo("loaded file with %d frames." % len(data))

    angle_range = 45.0

    for frame in data:
        msg = pau()

        msg.m_headRotation.x = (frame[0] * 2) - 1
        msg.m_headRotation.y = (frame[1] * 2) - 1
        msg.m_headRotation.z = (frame[2] * 2) - 1
        msg.m_headRotation.w = (frame[3] * 2) - 1
   
        msg.m_headTranslation.x = frame[4]
        msg.m_headTranslation.y = frame[5]
        msg.m_headTranslation.z = frame[6]
   
        msg.m_neckRotation.x = (frame[7] * 2) - 1
        msg.m_neckRotation.y = (frame[8] * 2) - 1
        msg.m_neckRotation.z = (frame[9] * 2) - 1
        msg.m_neckRotation.w = (frame[10] * 2) - 1
   
        msg.m_eyeGazeLeftPitch = (frame[11]*angle_range*2)-angle_range
        msg.m_eyeGazeLeftYaw = (frame[12]*angle_range*2)-angle_range
   
        msg.m_eyeGazeRightPitch = (frame[13]*angle_range*2)-angle_range
        msg.m_eyeGazeRightYaw = (frame[14]*angle_range*2)-angle_range

        msg.m_coeffs = frame[15:-2]

        #msg.m_shapekeys = ['eye-flare.UP.R', 'lips-narrow.L', 'lips-frown.R', 'eye-blink.UP.R', 'lip-UP.L.UP', 'eye-blink.UP.L', 'lips-frown.L', 'lips-narrow.R', 'eye-flare.UP.L', 'lip-DN.C.UP', 'eye-flare.LO.R', 'lip-DN.R.UP', 'brow_inner_UP.R', 'brow_outer_UP.L', 'brow_inner_UP.L', 'eye-flare.LO.L', 'brow_center_DN', 'lips-smile.R', 'lip-JAW.DN', 'lip-DN.R.DN', 'wince.L', 'lips-smile.L', 'eye-blink.LO.R', 'lip-UP.R.UP', 'lip-UP.C.DN', 'eye-blink.LO.L', 'brow_center_UP', 'lip-DN.L.DN', 'lip-DN.L.UP', 'wince.R', 'sneer.L', 'lips-wide.L', 'brow_outer_DN.R', 'lip-UP.R.DN', 'brow_inner_DN.L', 'brow_outer_up.R', 'brow_inner_DN.R', 'lip-DN.C.DN', 'lip-UP.L.DN', 'brow_outer_DN.L', 'lip-UP.C.UP', 'lips-wide.R', 'sneer.R']
        
        msg.m_shapekeys = ['EyeBlink_L', 'EyeBlink_R', 'EyeSquint_L', 'EyeSquint_R', 'EyeDown_L', 'EyeDown_R', 'EyeIn_L', 'EyeIn_R', 'EyeOpen_L', 'EyeOpen_R', 'EyeOut_L', 'EyeOut_R', 'EyeUp_L', 'EyeUp_R', 'BrowsD_L', 'BrowsD_R', 'BrowsU_C_L', 'BrowsU_C_R', 'BrowsU_L', 'BrowsU_R', 'BrowsSqueeze_L', 'BrowsSqueeze_R', 'JawOpen', 'LipsTogether', 'JawLeft', 'JawRight', 'JawFwd', 'LipsUpperUp_L', 'LipsUpperUp_R', 'LipsLowerDown_L', 'LipsLowerDown_R', 'LipsUpperClose', 'LipsLowerClose', 'LipsLowerOpen', 'LipsUpperOpen', 'MouthSharpCornerPull_L', 'MouthSharpCornerPull_R', 'MouthSmile_L', 'MouthSmile_R', 'MouthDimple_L', 'MouthDimple_R', 'LipsStretch_L', 'LipsStretch_R', 'MouthFrown_L', 'MouthFrown_R', 'MouthPress_L', 'MouthPress_R', 'LipsPucker_L', 'LipsPucker_R', 'LipsFunnel_L', 'LipsFunnel_R', 'MouthLeft', 'MouthRight', 'ChinLowerRaise', 'ChinUpperRaise', 'Sneer_L', 'Sneer_R', 'Puff', 'CheekSquint_L', 'CheekSquint_R' ]

        rospy.loginfo("sending frame.")

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
