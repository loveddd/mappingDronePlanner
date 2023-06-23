import math
import os
import numpy as np
import matplotlib.pyplot as plt
import pandas as pds
import shapely
import geopandas as gpd
from geopy import distance as dis
import env
import dataFilter as df


class Crossing:
    def __init__(self,path_points) -> None:
        pts = [path_points[0],path_points[-1]]
        fltr = df.DataFilter([],
                    [],
                    pts,
                    env.path_road,
                    env.path_rail) 
        self.path_data = shapely.LineString(path_points)
        self.new_path = list(self.path_data.coords)
        self.roads_data = fltr.roads
        self.railways_data = fltr.railways
        self.length_road = 0.03  #km
        self.length_rail = 0.04  #km

    def pairs(self,lst):
        for i in range(1, len(lst)):
            yield lst[i-1], lst[i]

    def getCrossingPath(self,path,line_data,length:float):
        intscts = []
        pth = path
        for crs in line_data.geometry.values:
            seg_rd = []
            seg_pth = []

            if crs.intersects(shapely.LineString(pth)):
                if not type(crs)==shapely.LineString:
                    continue
                point = shapely.Point()
                points = crs.intersection(shapely.LineString(pth))
                # print(points)
                if(type(points) == shapely.MultiPoint):
                    point = points.geoms[0]
                else:
                    point = points

                duplicate = False

                if not intscts:
                    intscts.append(point)
                else:
                    for p in intscts:
                        if dis.distance(p.coords,point.coords).km < length*0.9:
                            duplicate = True
                            break

                if not duplicate:
                    intscts.append(point)
                    for pair in self.pairs(list(crs.coords)):
                        if shapely.LineString([pair[0],pair[1]]).distance(point)<1e-6:
                            seg_rd = shapely.LineString([pair[0],pair[1]])
                    for pair in self.pairs(list(pth)):
                        if shapely.LineString([pair[0],pair[1]]).distance(point)<1e-6:
                            seg_pth = [pair[0],pair[1]]
                    
                    path = self.generateCrossingPath(seg_rd,point,length)
                    
                    for idx,pt in enumerate(pth):
                        if(idx+1 >= len(pth)):
                            break
                        if pt == seg_pth[0] and pth[idx+1] == seg_pth[1]:
                            # print(idx)
                            if(math.hypot(path[0][0]-pt[0],path[0][1]-pt[1]) > math.hypot(path[1][0]-pt[0],path[1][1]-pt[1])):
                                pth.insert(idx+1,path[0])
                                pth.insert(idx+1,path[1])  
                            else:
                                pth.insert(idx+1,path[1])
                                pth.insert(idx+1,path[0])                            

                            break
        return pth

    def getPath(self):
        self.new_path = self.getCrossingPath(self.new_path,self.railways_data,self.length_rail)
        return self.getCrossingPath(self.new_path,self.roads_data,self.length_road)
                

    def generateCrossingPath(self,road:shapely.LineString,intersection:shapely.Point,length:float):
        if(not road.distance(intersection)<1e-8):
            return False
        x = (road.coords[1][1] - road.coords[0][1])*40075/360*math.cos(road.coords[0][0])
        y = (road.coords[1][0] - road.coords[0][0])*111.32
        alpha = math.atan2(y,x)
        # print(alpha)
        pt1_x = intersection.x - length*0.5*math.cos(alpha)*360/40075/math.cos(road.coords[0][0])
        pt1_y = intersection.y + length*0.5*math.sin(alpha)/111.32
        pt2_x = intersection.x + length*0.5*math.cos(alpha)*360/40075/math.cos(road.coords[0][0])
        pt2_y = intersection.y - length*0.5*math.sin(alpha)/111.32
        return [(pt1_x,pt1_y),(pt2_x,pt2_y)]
    
    






if __name__ == "__main__":

    path_to_path_planned = 'data/path_planned.csv'
    path_road = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_roads_free_1.shp"
    path_rail = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_railways_free_1.shp"

    pts = pds.read_csv(path_to_path_planned).values
    print([pts[0],pts[-1]])
    cs = Crossing(pts)

    pth = cs.getPath()
    df = pds.DataFrame(pth)
    df.to_csv("data/path_crossing.csv",index=False)

    # path_planned = pds.read_csv('data/path_planned.csv')
    # roads_data = gpd.read_file('data/temp/roads_clipped.shp')
    # roads = []
    # path = shapely.LineString(path_planned)
    # for i in roads_data.geometry.values:
    #     roads.append(i)



    # data = pds.read_csv('data/path_points.csv')
    # x_start = tuple(data.values[0])  # Starting node
    # print ("start: " + str(x_start))
    # x_goal = tuple(data.values[1])  # Goal node
    # print ("goal: " + str(x_goal))
    # step = max(abs(x_start[0]- x_goal[0]),abs(x_start[1]- x_goal[1]))/20.0
    # # print (step)

    # rrt_star = informed_rrt_star.IRrtStar(x_start, x_goal, step, 0.5, 12, 1000)
    # tic = time.perf_counter()

    # if not rrt_star.planning():
    #     print("Cannot plan path")
    # print("Planning algorithm execution time: " + str(time.perf_counter()-tic))