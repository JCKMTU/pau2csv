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
    pub = rospy.Publisher('/blender_api/set_pau', pau, queue_size=10)
    rospy.init_node('csv2pau', anonymous=True)

    rate = rospy.Rate(29) # 29fps approx. measure this exactly.

    data = np.loadtxt(sys.argv[1], delimiter=',')

    rospy.loginfo("loaded file with %d frames." % len(data))

    for frame in data:
        msg = pau()
        msg.m_coeffs = frame[:43]
        msg.m_shapekeys = ['eye-flare.UP.R', 'lips-narrow.L', 'lips-frown.R', 'eye-blink.UP.R', 'lip-UP.L.UP', 'eye-blink.UP.L', 'lips-frown.L', 'lips-narrow.R', 'eye-flare.UP.L', 'lip-DN.C.UP', 'eye-flare.LO.R', 'lip-DN.R.UP', 'brow_inner_UP.R', 'brow_outer_UP.L', 'brow_inner_UP.L', 'eye-flare.LO.L', 'brow_center_DN', 'lips-smile.R', 'lip-JAW.DN', 'lip-DN.R.DN', 'wince.L', 'lips-smile.L', 'eye-blink.LO.R', 'lip-UP.R.UP', 'lip-UP.C.DN', 'eye-blink.LO.L', 'brow_center_UP', 'lip-DN.L.DN', 'lip-DN.L.UP', 'wince.R', 'sneer.L', 'lips-wide.L', 'brow_outer_DN.R', 'lip-UP.R.DN', 'brow_inner_DN.L', 'brow_outer_up.R', 'brow_inner_DN.R', 'lip-DN.C.DN', 'lip-UP.L.DN', 'brow_outer_DN.L', 'lip-UP.C.UP', 'lips-wide.R', 'sneer.R']

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
