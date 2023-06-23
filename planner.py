import pandas as pds
import os,sys

import dataFilter
import informed_rrt_star as irs
import crossingOptimization as crs
import time
from geopy import distance


class Planner:
    def __init__(self,crs_opt = False,refuel_opt = False) -> None:
        self.crs_opt = crs_opt
        self.refuel_opt = refuel_opt
        
        pass

    def pointToPointPlanner(self,path_points):
        # IRrt* 
        x_start = tuple(path_points[0])  # Start node
        print ("start: " + str(x_start))
        x_goal = tuple(path_points[1])  # Goal node
        print ("goal: " + str(x_goal))
        step = max(abs(x_start[0]- x_goal[0]),abs(x_start[1]- x_goal[1]))/5.0

        tic = time.perf_counter()   
        rrt_star = irs.IRrtStar(x_start, x_goal, step, 0.10, 12, 1000)
        print("Data Filtering execution time: " + str(time.perf_counter()-tic))
        
        tic = time.perf_counter()
        if not rrt_star.planning():
            print("Cannot Plan path")
            return []
        else:
            path = rrt_star.rev_path
        print("Planning algorithm execution time: " + str(time.perf_counter()-tic))


        if self.refuel_opt:
            # do refuel planning
            pass
        
        if self.crs_opt:
            cs = crs.Crossing(path)
            path = cs.getPath()
        # output = rrt_star.rev_path

        return path

if __name__ == '__main__':
    path_to_pathpoint_file = "data/path_points.csv"
    path_points = pds.read_csv(path_to_pathpoint_file).values
    # print (path_points)
    planner = Planner(crs_opt=True,refuel_opt=False)
    path = []
    for i in range(1,len(path_points)):
        print("Line distance(km): " + str(distance.distance(path_points[i-1],path_points[i]).km))
        p = planner.pointToPointPlanner(path_points[i-1:i+1])
        if p:
            for pt in p:
                path.append(pt)
        else:
            print("Cannot plan path from " + str(path_points[i-1])+" to " + str(path_points[i+1]))
            break
    frame = pds.DataFrame(path)
    frame.to_csv("data/path_planned.csv",index=False)
    print(path)

    
