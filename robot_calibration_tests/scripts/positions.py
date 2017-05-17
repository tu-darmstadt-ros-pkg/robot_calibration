from __future__ import division
import numpy as np

wb = 0.01
cs = 0.05
csd2 = cs/2

s1 = np.array([-0.1667, 0.17837, 0.0])
s2 = np.array([-0.0124, 0.09285, 0.0])
s3 = np.array([-0.3444, -0.1807, 0.0])
s4 = np.array([-0.4432, -0.0949, 0.0])
lidarbox_height = 0.20
c1 = np.array([-0.3240, 0.05, lidarbox_height])
c2 = np.array([-0.49398, 0.049, lidarbox_height])

t = [0] * 5
t[0] = [s1[0] + wb + csd2, s2[1] + wb + csd2, 0.0]
t[1] = [s3[0] - wb - csd2, s4[1] - wb - csd2, 0.0]
t[2] = [c1[0] - wb - csd2, c1[1] + wb + csd2, c1[2]]
t[3] = [c2[0] + wb + csd2, c2[1], c2[2] - wb - csd2]
t[4] = [-0.09, 0, 0.045]

for i, tag in enumerate(t):
    s = "Tag: {}: [".format(i)
    for n in tag:
        s += "{:.4f}, ".format(n)
    print s + "]"
