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
import rospy
from pau2motors.msg import pau

def msg2str(data):
    num = "{:.20e},"

    angle_range = 45
    angle_factor = 1.0/(angle_range*2.0)

    output = ""

    output += num.format((data.m_headRotation.x + 1) / 2)
    output += num.format((data.m_headRotation.y + 1) / 2)
    output += num.format((data.m_headRotation.z + 1) / 2)
    output += num.format((data.m_headRotation.w + 1) / 2)

    # this is always 0 anyways, so no scaling for now.
    output += num.format(data.m_headTranslation.x)
    output += num.format(data.m_headTranslation.y)
    output += num.format(data.m_headTranslation.z)

    output += num.format((data.m_neckRotation.x + 1) / 2)
    output += num.format((data.m_neckRotation.y + 1) / 2)
    output += num.format((data.m_neckRotation.z + 1) / 2)
    output += num.format((data.m_neckRotation.w + 1) / 2)

    output += num.format((data.m_eyeGazeLeftPitch+angle_range)*angle_factor)
    output += num.format((data.m_eyeGazeLeftYaw+angle_range)*angle_factor)

    output += num.format((data.m_eyeGazeRightPitch+angle_range)*angle_factor)
    output += num.format((data.m_eyeGazeRightYaw+angle_range)*angle_factor)

    for coeff in data.m_coeffs:
      output += num.format(coeff) #already normalized

    output = output[:len(output)-1] #cut superflous comma

    return output

def callback(data):
    print(msg2str(data))

def header(concat_str=None):
    '''concat_str is an optional argument used to switch the headers of
    shapekey coefficients. If string is not passed the headers from
    src/faceshift_puppeteering/sophia/shapekey_pairing.json are used.
    '''
    header = "#"
    header += "m_headRotation.x,"
    header += "m_headRotation.y,"
    header += "m_headRotation.z,"
    header += "m_headRotation.w,"

    header += "m_headTranslation.x,"
    header += "m_headTranslation.y,"
    header += "m_headTranslation.z,"

    header += "m_neckRotation.x,"
    header += "m_neckRotation.y,"
    header += "m_neckRotation.z,"
    header += "m_neckRotation.w,"

    header += "m_eyeGazeLeftPitch,"
    header += "m_eyeGazeLeftYaw,"

    header += "m_eyeGazeRightPitch,"
    header += "m_eyeGazeRightYaw,"

    if concat_str:
        header += concat_str
    else:
        # TODO: extract these from first shapekeys msg.
        header += "eye-flare.UP.R, lips-narrow.L, lips-frown.R, eye-blink.UP.R, lip-UP.L.UP,"
        header += "eye-blink.UP.L, lips-frown.L, lips-narrow.R, eye-flare.UP.L, lip-DN.C.UP,"
        header += "eye-flare.LO.R, lip-DN.R.UP, brow_inner_UP.R, brow_outer_UP.L, brow_inner_UP.L,"
        header += "eye-flare.LO.L, brow_center_DN, lips-smile.R, lip-JAW.DN, lip-DN.R.DN, wince.L,"
        header += "lips-smile.L, eye-blink.LO.R, lip-UP.R.UP, lip-UP.C.DN, eye-blink.LO.L,"
        header += "brow_center_UP, lip-DN.L.DN, lip-DN.L.UP, wince.R, sneer.L, lips-wide.L,"
        header += "brow_outer_DN.R, lip-UP.R.DN, brow_inner_DN.L, brow_outer_up.R, brow_inner_DN.R,"
        header += "lip-DN.C.DN, lip-UP.L.DN, brow_outer_DN.L, lip-UP.C.UP, lips-wide.R, sneer.R"

    return header

def listener():
    print(header())

    rospy.init_node('pau2csv', anonymous=True)
    rospy.Subscriber("/blender_api/set_pau", pau, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
