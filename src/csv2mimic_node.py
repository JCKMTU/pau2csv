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
import math

def talker():
    pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    rospy.init_node('csv2pau', anonymous=True)

    rate = rospy.Rate(120) # 29fps approx. measure this exactly.

    data = np.loadtxt(sys.argv[1], delimiter=',', skiprows=1)

    rospy.loginfo("loaded file with %d frames." % len(data))

    angle_range = 45.0

    for frame in data:
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

        ## Delete superfluous frames:
        cut_frame = np.delete(frame, np.s_[14:28], axis=0)
	
	
	cut_frame[0:10] = np.power(cut_frame[0:10],5.8)
	cut_frame[12] *= 1.5
	cut_frame[13:] *= 4.5
	cut_frame = np.minimum(1.0, cut_frame)
	#cut_frame *= 6.0

	print(cut_frame[0:10])

	# eyelids
	cut_frame[17] +=0.1
	cut_frame[19] +=0.1
	cut_frame[13] = cut_frame[17]       
        cut_frame[15] = cut_frame[19]
	cut_frame[14] = 0.0
	cut_frame[16] = 0.0

	#jaw
	#cut_frame[12] *= 0.85
	#brows
	#cut_frame[0:9] *= 1.60


	#ee
	#cut_frame[22] = 0.0 
        #cut_frame[24] = 0.0

	#cut_frame[21] = cut_frame[33]
	#cut_frame[23] = cut_frame[34]
	#mouth
	#cut_frame[25:32] *= 0.8
        #cut_frame[35:41] *= 1.9

        msg.m_coeffs = cut_frame

        print("sending frame...")

        msg.m_shapekeys = ['brow_center_DN', 'brow_center_UP', 'brow_inner_DN.L', 'brow_inner_UP.L', 'brow_inner_DN.R', 'brow_inner_UP.R', 'brow_outer_DN.L', 'brow_outer_UP.L', 'brow_outer_DN.R', 'brow_outer_up.R', 'wince.L', 'wince.R', 'lip-JAW.DN', 'eye-blink.LO.L', 'eye-flare.LO.L', 'eye-blink.LO.R', 'eye-flare.LO.R', 'eye-blink.UP.L', 'eye-flare.UP.L', 'eye-blink.UP.R', 'eye-flare.UP.R', 'lips-wide.L', 'lips-narrow.L', 'lips-wide.R', 'lips-narrow.R', 'lips-frown.L', 'lips-frown.R', 'lip-DN.C.DN', 'lip-DN.C.UP', 'lip-DN.L.DN', 'lip-DN.L.UP', 'lip-DN.R.DN', 'lip-DN.R.UP', 'lips-smile.L', 'lips-smile.R', 'lip-UP.C.DN', 'lip-UP.C.UP', 'lip-UP.L.DN', 'lip-UP.L.UP', 'lip-UP.R.DN', 'lip-UP.R.UP', 'sneer.L', 'sneer.R']

        assert(len(msg.m_shapekeys) == len(cut_frame))

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
