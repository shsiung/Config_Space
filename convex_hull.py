import matplotlib.pyplot as plt
import numpy as np
import random
import sys

def turn(p, q, r):
    """Returns -1, 0, 1 if p,q,r forms a right, straight, or left turn."""
    return cmp((q[0] - p[0])*(r[1] - p[1]) - (r[0] - p[0])*(q[1] - p[1]), 0)

def dist(m, n):
    """Returns the squared Euclidean distance between m and n"""
    dx, dy = n[0] - m[0], n[1] - m[1]
    return dx * dx + dy * dy

def _next_hull_pt(points, p):
    """Returns the next point on the convex hull in CCW from p."""
    q = p
    for r in points:
        t = turn(p, q, r)
        if t == -1 or t == 0 and dist(p, r) > dist(p, q):
            q = r
    return q

def convex_hull(points):
    """Returns the points on the convex hull of points in CCW order."""
    hull = [min(points)]
    for p in hull:
        #print p
        q = _next_hull_pt(points, p)
        if q != hull[0]:
            hull.append(q)
    return hull

"""
if __name__ == "__main__":

    # Generate random points to test
    n_pts = int(sys.argv[1])
    pt_x = random.sample(xrange(-n_pts*3,n_pts*3),n_pts)
    pt_y = random.sample(xrange(-n_pts*3,n_pts*3),n_pts)
    points = zip(pt_x,pt_y)
    
    # Plot the results    
    fig = plt.figure()
    ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(2, 1, 2, sharex=ax1)
    ax1.plot(pt_x, pt_y,'ro',markersize=5)
    ax1.set_title("Data points")
    convex_pts = convex_hull(points)
    convex_x = [i[0] for i in convex_pts]
    convex_y = [i[1] for i in convex_pts]

    ax2.plot(pt_x,pt_y,'ro',markersize=5)
    ax2.plot(convex_x+[convex_x[0]], convex_y+[convex_y[0]],'o-',markersize=10)
    ax2.set_title("Convex sets")
    plt.show()
"""
