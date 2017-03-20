#!/usr/bin/env python
#
# Robot PAU to csv output
# Copyright (C) 2017 Hanson Robotics
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

import argparse
import os
import rosbag

from pau2csv_node import header, msg2str
from pau2motors.msg import pau

# Additional Parameters
# NOTE: Make sure the types are listed in alphabetic order. Why? To make
# testing of consistency between csv files output easier.
emotion_types = ["confident", "confused"]
emotion_enum_dict = dict((i, emotion_types.index(i)) for i in emotion_types)
interaction_types = ["listening", "speaking"]
interaction_enum_dict = \
    dict((i, interaction_types.index(i)) for i in interaction_types)

# Script defaults
default_topic = "/blender_api/faceshift_values"
output_dir =  os.getcwd() + '/output/'

def get_arguments():
    parser = argparse.ArgumentParser(description='Convert rosbag recordings \
        to csv files.')
    parser.add_argument('-f', '--file', type=str, required=True,
        help="Path to rosbag that is to be converted to csv file.")
    parser.add_argument('-e', '--emotion-type', type=str, required=True,
        metavar=None, choices=emotion_types,
        help='Choose the emotion-type of the model.')
    parser.add_argument('-i', '--interaction-type', type=str, required=True,
        metavar=None, choices=interaction_types,
        help='Choose the interaction-type of the model.')
    parser.add_argument('-o', '--output-dir', type=str, default=output_dir,
        metavar="", help='The directory where the csv files are outputed.')

    return parser.parse_args()


def main():
    args = get_arguments()

    # Check if the file passed is a rosbag and use its name for the output csv
    # file name.
    try:
        bag = rosbag.Bag(args.file)
        output_file_name = os.path.basename(args.file).split(".")[0] + ".csv"

        # Create the output dir if it doesn't exist
        if not os.path.exists(args.output_dir):
            print("Making directory %r" % args.output_dir)
            os.mkdir(args.output_dir)

        output_file_path = os.path.join(args.output_dir, output_file_name)

    except rosbag.bag.ROSBagException:
        print "%s does not appear to be a rosbag file." % args.file
        exit(1)

    write_header = True

    with open(output_file_path, 'w') as o:
        print("Starting writing message values to %r" % output_file_path)
        for topic, msg, time in bag.read_messages(topics=default_topic):
            # Write header
            if write_header:
                # NOTE The order should be consistent with new_msg below.
                new_header =  header(",".join(msg.m_shapekeys)) + \
                    ", emotion_type, interaction_type\n"
                o.write(new_header)
                write_header = False

            # Write the message values
            # NOTE: The order should match with new_header above.
            new_msg = msg2str(msg) + ",%d,%d\n" % \
                (emotion_enum_dict[args.emotion_type],
                interaction_enum_dict[args.interaction_type])

            o.write(new_msg)

    print("Finished writing message values to %r" % output_file_path)
    bag.close()

if __name__ == '__main__':
    main()
