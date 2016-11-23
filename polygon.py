from convex_hull import *

class Polygon:
    """ Class representing a polygon (obstacle)"""    
    def __init__(self, x=[], y=[]):
        self.points = zip(x, y)
        self.calc_convex_hull()

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
        
