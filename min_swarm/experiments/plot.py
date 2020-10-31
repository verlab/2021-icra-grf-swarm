#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
import scipy.stats
import sys
import os

def mean_confidence_interval(data, confidence=0.95):
    a = 1.0 * np.array(data)
    n = len(a)
    m, se = np.mean(a), scipy.stats.sem(a)
    h = se * scipy.stats.t.ppf((1 + confidence) / 2., n-1)
    return m, m-h, m+h



fig = plt.figure(dpi=100)
ax = fig.add_subplot(111)

file = sys.argv[1]
data = np.load(file, allow_pickle=True)
data = data["metric"]
print(data.shape)
print(data[-1,:])

mean = []
mean_u = []
mean_l = []
for i in range(data.shape[0]):
	m, l, u = mean_confidence_interval(data[i,:])
	mean.append(m)
	mean_l.append(l)
	mean_u.append(u)

x = range(data.shape[0])
ax.plot(x, mean_l, "--", color="black", linewidth=0.4)
ax.plot(x, mean_u, "--", color="black", linewidth=0.4)
ax.plot(x, mean, color="blue", label="0.5m", linewidth=0.8)

#-----------------------------
ax.set_title("Convergence rate per sensing range")
ax.set_xlabel('Iterations (logscale)')
ax.set_ylabel('# Cluster')
ax.set_xscale('symlog')
plt.show()