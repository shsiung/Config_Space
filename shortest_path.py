from polygon import *
from robot import *
from a_star import *
import numpy as np
import math

def dist(m, n):
    """Returns the squared Euclidean distance between m and n"""
    dx, dy = n[0] - m[0], n[1] - m[1]
    return math.sqrt(dx * dx + dy * dy)

def dotproduct(v1, v2):
    """Dot product of two vectors"""
    return v1[0]*v2[0]+v1[1]*v2[1]

def length(v):
    """Length of a vector"""
    return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
    """Returns the angle between two vector"""
    return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

def inside_polygon(point, polygon):
    """ 
    This vector will return true if the point is inside 
    the given polygon (if angle from the current point to 
    all corners sum to 360 degree), and false otherwise.
    """
    ang = 0;
    vector = []
    for poly in polygon:
        vector += [(poly[0]-point[0], poly[1]-point[1])]

    for i in range(len(vector)-1):
        ang += angle(vector[i],vector[i+1])     
    
    return ang == 2*math.pi

def find_config_polygon(polygon, robot):
    """Find the polygon in configuration space"""
    new_poly_points = []
    max_dist = 0
    config_origin = robot.points[0]
    for poly_pt in polygon.points:
        touch = False
        for robot_pt in robot.points:
            x_diff = poly_pt[0]-robot_pt[0]
            y_diff = poly_pt[1]-robot_pt[1]
            for robot_pt in robot.points:
                if inside_polygon(robot_pt, polygon.points):
                    touch = True
            if not touch:
                new_poly_points = new_poly_points + [(config_origin[0]+x_diff, config_origin[1]+y_diff)]

    polygon.set_points(new_poly_points)
    polygon.update()
    return polygon

def line_eq(pt1, pt2):
    """ Fine the line equation y=mx+b given two end points"""
    denom = (pt2[0]-pt1[0])*1.0
    num = (pt2[1]-pt1[1])*1.0
    if denom == 0:
        return (1,0,pt2[0])
    elif num == 0:
        return (0,1,-pt2[1])
    else:
        m = num/denom
        q = pt1[1]-m*pt1[0]
        return (m, 1, -q)

def intersect(v1, v2):

    """ Input two vectors: v1, v2, and each has a form (pt1, pt2)"""
    L1 = line_eq(v1[0],v1[1])
    L2 = line_eq(v2[0],v2[1])
    tol = 0.0000001

    denom =  L1[0]*L2[1]-L2[0]*L1[1]
    # Teo lines are parallel
    if denom == 0:
        if (max(v1[0][0],v1[1][0]) < max(v2[0][0],v2[1][0]) and \
           max(v1[0][0],v1[1][0]) > min(v2[0][0],v2[1][0])) or \
           (min(v1[0][0],v1[1][0]) < max(v2[0][0],v2[1][0]) and\
           min(v1[0][0],v1[1][0]) > min(v2[0][0],v2[1][0])):
            return (0,0)
        else:
            return False
    else:
        x = (L1[2]*L2[1]-L2[2]*L1[1])/denom
        y = (L1[2]*L2[0]-L2[2]*L1[0])/denom
        distL1 = dist((x,y),v1[0]) + dist((x,y),v1[1])
        distL2 = dist((x,y),v2[0]) + dist((x,y),v2[1])
        # return (x,y)

        if distL1 <= dist(v1[0],v1[1])+tol and distL2 <= dist(v2[0],v2[1])+tol:
            return (x,y)
        else:
            return False

def calc_valid_edge(polys, total_nodes, total_poly_edges):
    valid_edge = []
    for poly in polys:
        for node_start in poly.nodes:
            for node_end in total_nodes:
                if node_end in poly.nodes:
                    continue
                else:
                    does_intersect = False
                    for poly_seg in total_poly_edge:
                        if node_end.node in poly_seg or node_start.node in poly_seg:
                            continue
                        else:
                            intersection = intersect((node_start.node,node_end.node),poly_seg)
                            if intersection == False:
                                continue
                            else:   
                                if intersection == node_end.node or intersection == node_end.node:
                                    continue
                                else:
                                    does_intersect = True
                    if not does_intersect:
                        node_start.neighbor += [node_end]
                        node_end.neighbor += [node_start]
                        valid_edge += [(node_start.node,node_end.node)]

    valid_edge += total_poly_edge
    return valid_edge

def check_overlap_two(poly1, poly2):
    for corner1 in poly1.nodes:
        for poly_seg in poly2.line_segs:
            if intersect((poly1.inner_point,corner1.node),poly_seg):
                return True
    for corner2 in poly2.nodes:
        for poly_seg in poly1.line_segs:
            if intersect((poly2.inner_point,corner2.node),poly_seg):
                return True
    print "HIII"
    return False

def check_overlap(polys):
    new_polys = []

    for poly1 in polys:
        check_again = False
        for poly2 in polys:
            if poly1 != poly2 and check_overlap_two(poly1,poly2):  
                new_poly = Polygon(poly1.x+poly2.x, poly1.y+poly2.y)
                new_poly.update()
                new_polys += [new_poly]
                check_again = True;
                print "OVERLAP"

        if not check_again:
            new_polys += [poly1]

    if check_again:
        new_polys = check_overlap(new_polys)

    return new_polys



def random_env_generator(n_polys):
    centers_x = np.random.normal(0,50,n_polys+2)
    centers_y = np.random.normal(0,50,n_polys+2)

    centers = zip(centers_x, centers_y)
    polys = []
    num_pts = 20
    for poly in centers[0:-2]:
        poly_pts_x = np.random.normal(poly[0],10,num_pts)
        poly_pts_y = np.random.normal(poly[1],10,num_pts)
        polys += [Polygon(poly_pts_x,poly_pts_y)]

    robot_pts_x = np.random.normal(centers[-2][0],3,4);
    robot_pts_y = np.random.normal(centers[-2][1],3,4);
    robot = Robot(robot_pts_x, robot_pts_y);

    start = Node(robot.points[0])
    goal = Node(centers[-1])

    print centers
    return polys, robot, start, goal

if __name__ == "__main__":

    # Randomly generated environment
    [polys, robot, start, goal] = random_env_generator(int(sys.argv[1]))    

    # Initialization
    robot = Robot([8,10,10],[-1,-1,3])
    # start = Node(robot.points[0])
    # goal = Node((0,35))
    # poly2 = Polygon([14, 14, 19, 19, 15], [17,19,17,19, 21])
    # poly3 = Polygon([-2,6,15,2,4],[5,4,9,1,8])
    # poly4 = Polygon([-5,0,9,10,7],[15,24,28,23,30])
    # polys = [poly2, poly3, poly4]

    # # Plot the results    
    fig = plt.figure()
    ax2 = fig.add_subplot(1, 1, 1)

    for poly in polys:
        ax2.plot(poly.x+[poly.x[0]], poly.y+[poly.y[0]],'yo--',markersize=10)
        poly = find_config_polygon(poly,robot)
        ax2.plot(poly.x+[poly.x[0]], poly.y+[poly.y[0]],'yo-',markersize=10,linewidth=5)

    # polys = check_overlap(polys)
    print polys
    print "HERE"
    ax2.scatter(start.node[0],start.node[1],s=500,c='g',marker='o')
    ax2.scatter(goal.node[0],goal.node[1],s=500,c='r',marker='*')
    ax2.plot(robot.x+[robot.x[0]], robot.y+[robot.y[0]],'c-')
    ax2.set_title("Convex sets")

    total_nodes = [goal, start] # Total number of nodes in the graph
    total_poly_edge = []       # Total polygon edges in the graph

    for poly in polys:
        total_poly_edge += poly.line_segs
        total_nodes += poly.nodes

    valid_edge = calc_valid_edge(polys, total_nodes, total_poly_edge)

    for edge in valid_edge:
        ax2.plot([edge[0][0],edge[1][0]],[edge[0][1],edge[1][1]],'b--')

    shortest_path = a_star(start,goal,total_nodes)

    for i in range(len(shortest_path)-1):
        ax2.plot([shortest_path[i].node[0],shortest_path[i+1].node[0]],
                 [shortest_path[i].node[1],shortest_path[i+1].node[1]],'ro-',lw=5)

    plt.show()





