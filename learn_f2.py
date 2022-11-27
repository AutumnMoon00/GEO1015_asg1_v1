import random
import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

# dd, ii = KDTree.query()
#
with open('D:\\Geomatics\\Geo1015 DTM\\asgns\\1_v1\\python\\samples.xyz', 'r') as ptfile:
    lines = ptfile.readlines()
    ln_strip = list(map(str.strip, lines))
    ln_split = list(map(str.split, ln_strip))
    points = ln_split[1:]
    array = np.array([list(map(float, i)) for i in points])
    x = array[:, 0]
    y = array[:, 1]
    z = array[:, 2]

tree = KDTree(np.c_[x, y])
px = 125; py = 125
p = np.array([px, py])
dd, ii = tree.query([p], k=1)

z_elv = z[ii]
print(p, z_elv)

# print(x.shape)
# print(y.shape)
# print(x[200:300] - y[200:300])

plt.plot(x, y, 'o')
plt.plot(p[0], p[1], '+')
plt.show()
