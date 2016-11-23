class Robot:
    """ Class representing a robot"""
    def __init__(self, x=[], y=[]):
        self.points = zip(x,y)
        self.x = x
        self.y = y
