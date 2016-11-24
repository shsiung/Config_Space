from polygon import *
from robot import *
import math

def dist(m, n):
    """Returns the squared Euclidean distance between m and n"""
    dx, dy = n[0] - m[0], n[1] - m[1]
    return dx * dx + dy * dy

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
    polygon.calc_convex_hull()
    return polygon

    

if __name__ == "__main__":
    # Generate random points to test
    n_polys = int(sys.argv[1])
    n_pts = 5
    polys = []
 
    #for i in range(n_polys):
    #    pt_x = random.sample(xrange(-n_pts*3,n_pts*3),n_pts)
    #    pt_y = random.sample(xrange(-n_pts*3,n_pts*3),n_pts)
    #    poly = poly + [Polygon(pt_x,pt_y)]
   
    start = (8,4)
    end = (4,14)
    robot = Robot([8,10,10],[4,4,7])

    poly2 = Polygon([14, 14, 19, 19, 15], [14,19,14,19, 21])
    poly3 = Polygon([6,8,2,4],[4,9,1,8])
    # Plot the results    
    fig = plt.figure()
    #ax1 = fig.add_subplot(2, 1, 1)
    ax2 = fig.add_subplot(1, 1, 1)
    #ax1.plot(pt_x, pt_y,'ro',markersize=5)
    #ax1.set_title("Data points")
    polys = [poly2, poly3]
    for poly in polys:
        ax2.plot(poly.x+[poly.x[0]], poly.y+[poly.y[0]],'bo-',markersize=10)
        poly = find_config_polygon(poly,robot)
        ax2.plot(poly.x+[poly.x[0]], poly.y+[poly.y[0]],'yo-',markersize=10)
    ax2.scatter(start[0],start[1],s=200,c='g',marker='o')
    ax2.scatter(end[0],end[1],s=200,c='r',marker='*')
    ax2.plot(robot.x+[robot.x[0]], robot.y+[robot.y[0]],'c-')
    ax2.set_title("Convex sets")
    plt.show()



