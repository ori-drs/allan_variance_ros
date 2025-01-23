#!/usr/bin/env python3
# Taken from http://wiki.ros.org/rosbag/Cookbook

"""
@file   cookbag.py
@brief  Script to reorganize a ROS bag file by message timestamps.
@author Russell Buchanan
"""

import rosbag
import argparse

parser = argparse.ArgumentParser()

parser.add_argument("--input", type=str, default=None)
parser.add_argument("--output", type=str, default=None)

args = parser.parse_args()

with rosbag.Bag(args.output, 'w', compression='lz4') as outbag:
    for topic, msg, t in rosbag.Bag(args.input).read_messages():
        # This also replaces tf timestamps under the assumption 
        # that all transforms in the message share the same timestamp
        if topic == "/tf" and msg.transforms:
            outbag.write(topic, msg, msg.transforms[0].header.stamp)
        else:
            outbag.write(topic, msg, msg.header.stamp if msg._has_header else t)
