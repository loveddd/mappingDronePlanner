"""
utils for collision check
@author: huiming zhou
"""

import math
import numpy as np
import os
import sys
import shapely
import geopandas as gpd
import matplotlib.pyplot as plt
import env
import pandas as pds

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None

    def toPoint(self):
        return [self.x,self.y]

class Utils:
    def __init__(self,path_points):
        self.env = env.Env(path_points)

        self.delta = 0
        self.obs_boundary = self.env.obs_boundary
        self.obs_polygon_fsb = self.env.obs_polygon_fsb
        self.obs_polygon_byzone = self.env.obs_polygon_byzone


    def update_obs(self, obs_bound,obs_polygon_fsb,obs_polygon_byzone):
        self.obs_boundary = obs_bound
        self.obs_polygon_fsb = obs_polygon_fsb
        self.obs_polygon_byzone = obs_polygon_byzone

    def is_collision(self, start, end) -> bool:
        if self.is_inside_obs(start) or self.is_inside_obs(end):
            return True

        line = shapely.LineString([start.toPoint(),end.toPoint()])

        if line.intersects(self.obs_boundary):
            return True

        for polygon in self.obs_polygon_byzone:
            if line.intersects(polygon):
                return True
        

        for polygon in self.obs_polygon_fsb:
            if line.intersects(polygon):
                return True
    
        return False

    def is_inside_obs(self, node):
        delta = self.delta
        for polygon in self.obs_polygon_fsb:
            if polygon.contains(shapely.Point(node.x,node.y)):
                return True

        for polygon in self.obs_polygon_byzone:
            if polygon.contains(shapely.Point(node.x,node.y)):
                return True

        if self.obs_boundary.contains(shapely.Point(node.x,node.y)):
            return True

        return False

    @staticmethod
    def get_ray(start, end):
        orig = [start.x, start.y]
        direc = [end.x - start.x, end.y - start.y]
        return orig, direc

    @staticmethod
    def get_dist(start, end):
        return math.hypot(end.x - start.x, end.y - start.y)

if __name__ == '__main__':

    path_points = pds.read_csv(env.path_to_pathpoint_file).values
    utils = Utils(path_points)
    p1= gpd.GeoSeries(utils.obs_boundary)
    p1.plot()
    plt.show(block = False)

