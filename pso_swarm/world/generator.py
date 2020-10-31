#!/usr/bin/env python3

import numpy as np
import math
import os
import sys
import math
import argparse
import sys

parser = argparse.ArgumentParser(description='World generation.')
parser.add_argument('--robots', help='Number of robots.', default=18, type=int)
parser.add_argument('--groups', help='Number of groups.', default=1, type=int)
parser.add_argument('--world', help='Size of the world SxS.', default=5, type=int)
parser.add_argument('--sensing', help='Sensing range.', default=0.5, type=float)
parser.add_argument('--seed', help='Random SEED.', default=1, type=int)
args = parser.parse_args()

np.random.seed(args.seed)

# Instanciate the robots and it initial velocities
q = 1.8 * args.world * (np.random.uniform(low=0.0, high=1.0, size=(args.robots, 2)) - 0.5) # position
g = 1.8 * args.world * (np.random.uniform(low=0.0, high=1.0, size=(args.robots, 2)) - 0.5) # position

r = args.robots/args.groups
for i in range(0, int(args.robots)):
	end = '\n'
	if i == (int(args.robots) - 1):
		end = ''
	print("{} {} {} {} {}".format(q[i,0], q[i,1], g[i,0], g[i,1], math.floor(i/r)), end = end)

