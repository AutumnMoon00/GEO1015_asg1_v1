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


def nn_xy(dt, kd, all_z, x, y):
    """
    !!! TO BE COMPLETED !!!

    Function that interpolates with nearest neighbour method.

    Input:
        dt:     the DT of the input points (a startinpy object)
        kd:     the kd-tree of the input points
        all_z:  an array with all the z values, same order as kd.data
        x:      x-coordinate of the interpolation location
        y:      y-coordinate of the interpolation location
    Output:
        z: the estimation of the height value,
           (raise Exception if outside convex hull)
    """
    # -- kd-tree docs: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.KDTree.html#scipy.spatial.KDTree
    # -- you are *not* allowed to use the function for the nn interpolation that I wrote for startinpy
    # -- you need to write your own code for this step
    z = random.uniform(0, 100)

    pt = np.array([float(x), float(y)])

    if not in_convexhull(dt, pt):
        raise Exception("Outside convex hull")

    dd, ii = kd.query([pt], k=1)
    z = all_z[ii]
    # print(all_z)

    return z


def main():
    # -- read the needed parameters from the file 'params.json' (must be in same folder)
    try:
        jparams = json.load(open('params.json'))
    except:
        print("ERROR: something is wrong with the params.json file.")
        sys.exit()
    # -- store the input 3D points in a DT
    dt = startinpy.DT()
    # -- cleaning of duplicates done in the process with tolerance of 1cm
    dt.snap_tolerance = 0.01
    with open(jparams['input-file']) as csvfile:
        r = csv.reader(csvfile, delimiter=' ')
        header = next(r)
        totall = 0
        for line in r:
            p = list(map(float, line))  # -- convert each str to a float
            assert (len(p) == 3)
            dt.insert_one_pt(p[0], p[1], p[2])
            totall += 1
        if totall > dt.number_of_vertices():
            print("INFO: {} duplicate points were removed".format(totall - dt.number_of_vertices()))
    # -- fetch all the (clean) points (see https://startinpy.readthedocs.io/en/latest/api.html#startinpy.DT.points)
    pts = dt.points[1:]
    # -- construct a KD-tree also, for fast nearest neighbours queries
    kd = scipy.spatial.KDTree(pts[:, :2])
    all_z = pts[:, -1]
    # -- find bbox, we get bbox[minx,miny,maxx,maxy]
    bbox = dt.get_bbox()

    ## my code
    # print(pts)
    # print(all_z)
    pts = dt.points
    trs = dt.triangles
    plt.triplot(pts[:, 0], pts[:, 1], trs)

    bbox = dt.get_bbox()

    print(pts.shape)
    print(trs.shape)
    one_triangle = trs[22]
    # print(one_triangle + np.array([one_triangle[0]]))
    first_vertex = one_triangle[0]
    second_vertex = one_triangle[1]
    third_vertex = one_triangle[2]
    print(one_triangle)
    print(first_vertex)
    print(second_vertex)
    print(third_vertex)
    print(dt.points[first_vertex])
    # for tri in trs:
    #     print(tri)

    px = 245
    py = 125
    p = np.array([px, py])

    print(in_convexhull(dt, p))
    print(nn_xy(dt, kd, all_z, px, py))

    # -- the vertex "0" shouldn't be plotted, so start at 1
    plt.plot(pts[1:, 0], pts[1:, 1], 'o')
    plt.plot(px, py, '+')
    plt.show()




main()
