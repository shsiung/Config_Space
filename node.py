class Node:
    def __init__(self, point):
        self.node = point;
        self.gScore = 0;
        self.fScore = 0;
        self.neighbor = [];
        self.cameFrom = [];

