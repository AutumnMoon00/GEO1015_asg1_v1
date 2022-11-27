import sys
import math
import random
import csv
import json
import time
import matplotlib.pyplot as plt

import numpy as np
import scipy.spatial  # -- for kd-tree & nearest neighbour(s) queries
import startinpy  # -- for a Delaunay triangulation

import raster


# import my_code_hw01

def dist_2pts(pt1, pt2):
    dist = math.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)
    return dist


def side_test(x1, y1, x2, y2, xp, yp):
    d = ((xp - x1) * (y2 - y1)) - ((yp - y1) * (x2 - x1))
    if d < 0:
        return -1
    elif d > 0:
        return 1
    elif d == 0:
        return 0


def in_triangle(pts, triangle, pt):
    """
    pts = points of the delaunay triangle
    """
    # pts = dt.points
    v1 = triangle[0]
    x1 = pts[v1][0]
    y1 = pts[v1][1]
    v2 = triangle[1]
    x2 = pts[v2][0]
    y2 = pts[v2][1]
    v3 = triangle[2]
    x3 = pts[v3][0]
    y3 = pts[v3][1]

    xp = pt[0]
    yp = pt[1]

    vertices = [v1, v2, v3, v1]
    stest = 0
    stest += side_test(x1, y1, x2, y2, xp, yp)
    stest += side_test(x2, y2, x3, y3, xp, yp)
    stest += side_test(x3, y3, x1, y1, xp, yp)

    if (stest == 1) or (stest == -1):
        return False
    else:
        return True


def in_convexhull(dt, pt):
    """
    dt = delaunay triangle
    pt = query point to check whether its in triangle or not

    return = True or False
    """
    trs = dt.triangles
    pts = dt.points
    for triangle in trs:
        if in_triangle(pts, triangle, pt):
            return True

    return False


def idw_xy(dt, kd, all_z, x, y, power, radius):
    """
    !!! TO BE COMPLETED !!!

    Function that interpolates with IDW

    Input:
        dt:     the DT of the input points (a startinpy object)
        kd:     the kd-tree of the input points
        all_z:  an array with all the z values, same order as kd.data
        x:      x-coordinate of the interpolation location
        y:      y-coordinate of the interpolation location
        power:  power to use for IDW
        radius: search radius
¨    Output:
        z: the estimation of the height value,
           (raise Exception if (1) outside convex hull or (2) no point in search radius
    """
    # -- kd-tree docs: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
    z = random.uniform(0, 100)
    pt = np.array([float(x), float(y)])
    if not in_convexhull(dt, pt):
        raise Exception("Outside convex hull")

    pts_in_r = sorted(kd.query_ball_point(pt, radius))
    if len(pts_in_r) == 0:
        raise Exception("No point in search radius")

    pts = dt.points[1:]
    num = 0
    den = 0
    for npts in pts_in_r:
        w = dist_2pts(pt, pts[npts][:2])
        num += w * all_z[npts]
        den += w ** (-1*power)

    z = num/den
    return z

