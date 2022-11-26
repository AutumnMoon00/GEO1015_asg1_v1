import numpy as np
from scipy.spatial import KDTree
import matplotlib.pyplot as plt

rng = np.random.default_rng()
points1 = rng.random((5, 2))
points2 = rng.random((5, 2))

npts = points1.shape[0]
colors = np.random.rand(npts)
plt.scatter(points1[:, 0], points1[:, 1])
plt.show()
