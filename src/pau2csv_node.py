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

def callback(data):

    #foo.m_eyeGazeLeftPitch   
    #foo.m_eyeGazeLeftYaw     
    #foo.m_eyeGazeRightPitch  
    #foo.m_eyeGazeRightYaw    

    #foo.m_headRotation       Quaternion
    #foo.m_headTranslation    Vector3
    #foo.m_neckRotation       Quaternion

    #foo.m_shapekeys
    #foo.m_coeffs 

    ## for now just output the m_coeffs
    #rospy.loginfo(rospy.get_caller_id() + "I heard a frame")
    output = ""
    for coeff in data.m_coeffs:
      output += "{:.20e},".format(coeff)
    output = output[:len(output)-1]

    #print(output)
    print(data)
    
def listener():

    rospy.init_node('pau2csv', anonymous=True)

    rospy.Subscriber("/blender_api/set_pau", pau, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
