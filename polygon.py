from convex_hull import *
from node import *

class Polygon:
    """ Class representing a polygon (obstacle)"""    
    def __init__(self, x=[], y=[]):
        self.points = zip(x, y)
        self.update()

    def set_xy(self,x,y):
        self.x = x
        self.y = y
        self.points = zip(self.x, self.y)

    def set_points(self, points):
        self.points = points

    def calc_convex_hull(self):
        self.points = convex_hull(self.points)
        self.x = [i[0] for i in self.points]
        self.y = [i[1] for i in self.points]
        self.set_xy(self.x, self.y)

    def calc_line_segs(self):
        self.line_segs=[]
        for i in range(len(self.x)):
            if i == len(self.x)-1:
                self.line_segs += [((self.x[i],self.y[i]),(self.x[0],self.y[0]))]
            else:
                self.line_segs += [((self.x[i],self.y[i]),(self.x[i+1],self.y[i+1]))]

    def calc_neighbors(self):
        for i in range(len(self.nodes)-1):
            self.nodes[i].neighbor += [self.nodes[i+1]]
            self.nodes[i+1].neighbor += [self.nodes[i]]

        self.nodes[-1].neighbor += [self.nodes[0]]
        self.nodes[0].neighbor += [self.nodes[-1]]

    def update(self):
        self.calc_convex_hull()
        self.calc_line_segs()
        self.nodes = map(lambda x: Node(x), self.points)
        self.inner_point = (sum(self.x)/len(self.x), sum(self.y)/len(self.y))
        self.calc_neighbors()
 




        
