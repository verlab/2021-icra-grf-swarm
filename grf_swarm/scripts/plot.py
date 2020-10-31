#!/usr/bin/env python3
import sys
import os

import rosbag
from std_msgs.msg import Float32


bag = rosbag.Bag(sys.argv[1])

for topic, msg, t in bag.read_messages(topics=['consensusVel', 'mClu']):
   print(msg)