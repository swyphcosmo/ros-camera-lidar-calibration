#!/usr/bin/python
"""
Copyright (c) 2012,
Systems, Robotics and Vision Group
University of the Balearican Islands
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Systems, Robotics and Vision Group, University of 
      the Balearican Islands nor the names of its contributors may be used to 
      endorse or promote products derived from this software without specific 
      prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""


PKG = 'bag_tools' # this package name

import roslib; roslib.load_manifest(PKG)
import rospy
import rosbag
import os
import sys
import argparse
import yaml
import sensor_msgs.msg

def change_camera_info(inbag,outbag,replacements):
  rospy.loginfo('      Processing input bagfile: %s', inbag)
  rospy.loginfo('     Writing to output bagfile: %s', outbag)
  # parse the replacements
  maps = {}
  for k, v in replacements.items():
    rospy.loginfo('Changing topic %s to contain following info (header will not be changed):\n%s',k,v)

  outbag = rosbag.Bag(outbag,'w')
  for topic, msg, t in rosbag.Bag(inbag,'r').read_messages():
    if topic in replacements:
      new_msg = replacements[topic]
      new_msg.header = msg.header
      msg = new_msg
    outbag.write(topic, msg, t)
  rospy.loginfo('Closing output bagfile and exit...')
  outbag.close();

def replacement(replace_string):
  pair = replace_string.split('=', 1)
  if len(pair) != 2:
    raise argparse.ArgumentTypeError("Replace string must have the form /topic=calib_file.yaml")
  if pair[0][0] != '/':
    pair[0] = '/'+pair[0]
  stream = file(pair[1], 'r')
  calib_data = yaml.load(stream)
  cam_info = sensor_msgs.msg.CameraInfo()
  cam_info.width = calib_data['image_width']
  cam_info.height = calib_data['image_height']
  cam_info.K = calib_data['camera_matrix']['data']
  cam_info.D = calib_data['distortion_coefficients']['data']
  cam_info.R = calib_data['rectification_matrix']['data']
  cam_info.P = calib_data['projection_matrix']['data']
  cam_info.distortion_model = calib_data['distortion_model']
  return pair[0], cam_info

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Change camera info messages in a bagfile.')
  parser.add_argument('inbag', help='input bagfile')
  parser.add_argument('outbag', help='output bagfile')
  parser.add_argument('replacement', type=replacement, nargs='+', help='replacement in form "TOPIC=CAMERA_INFO_FILE", e.g. /stereo/left/camera_info=my_new_info.yaml')
  args = parser.parse_args()
  replacements = {}
  for topic, calib_file in args.replacement:
    replacements[topic] = calib_file
  try:
    change_camera_info(args.inbag, args.outbag, replacements)
  except Exception, e:
    import traceback
    traceback.print_exc()
