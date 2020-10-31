#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np


fig = plt.figure(figsize=(10,8), dpi=100)
ax = fig.add_subplot(111, aspect=1.0)
# ax = plt.Axes(fig, [0., 0., 1., 1.])
# ax.set_axis_off()
# fig.add_axes(ax)
# ax.imshow(im_np, aspect='normal')
const = 1.0

# Cicle
r = 2.5
x = 1
y = 2
circle = plt.Circle((x, y), r*const, color='r', fill=False)
ax.add_artist(circle)


# Line
x1 = 2.5 - x
y1 = 2.5 - y
x2 = -5 - x
y2 =  5 - y
ax.plot([x1+x,x2+x], [y1+y,y2+y], color="b")



# function
dx = x2 - x1
dy = y2 - y1
dr = np.sqrt(dx*dx + dy*dy)
D = x1*y2 - x2*y1

disc = (r*r)*(dr*dr) - (D*D)
print(disc)

if disc < 0:
	print("no intersection")
elif disc == 0:
	print("tangent")
else:
	x1 = (D*dy + np.sign(dy) * dx * np.sqrt(disc)) / (dr*dr)
	x2 = (D*dy - np.sign(dy) * dx * np.sqrt(disc)) / (dr*dr)
	y1 = (-D*dx + np.abs(dy) * np.sqrt(disc)) / (dr*dr)
	y2 = (-D*dx - np.abs(dy) * np.sqrt(disc)) / (dr*dr)
	print(x1,y1, x2, y2)
	ax.plot(x1+x,y1+y, "o")
	ax.plot(x2+x,y2+y, "o")



ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])	
ax.set_aspect('equal')
ax.autoscale_view()
ax.grid(color='gray', linestyle='-', linewidth=0.1)
plt.show()