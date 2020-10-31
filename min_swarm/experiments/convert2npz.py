#!/usr/bin/env python3

import numpy as np
import math
import os
import sys
import math
import argparse
import sys
from tqdm import tqdm

import glob


parser = argparse.ArgumentParser(description='Convert log to npz and compute metrics.')
parser.add_argument('--path', help='Set the folder witch contains the logs files.', type=str)
args = parser.parse_args()

data = []
ncol = 0
nrow = 0
for filename in glob.glob(args.path+"/*.log"):
	text = None
	print(filename)
	with open(filename, 'r') as reader:
		text = reader.read()
		text = text.split("\n")
	if "," in text[0]:
		del text[0]
	print(text[0])
	pbar = tqdm(range(0, len(text)-1))
	data_ = []
	for i in pbar:
		data_.append(int(text[i]))
	data.append(data_)
	ncol += 1
	nrow = len(data_)

data = np.array(data).reshape(ncol,nrow).transpose() 
print(data[-1,:])
np.savez_compressed(args.path+".npz", metric=data)