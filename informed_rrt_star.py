
import os
import sys
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as Rot
import matplotlib.patches as patches
import pandas as pds
import time
import env,utils
from utils import Node
from shapely import LineString


class IRrtStar:
    def __init__(self, x_start, x_goal, step_len,
                 goal_sample_rate, search_radius, iter_max):
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.search_radius = search_radius
        self.iter_max = iter_max

        self.env = env.Env([x_start,x_goal])
        #self.plotting = plotting.Plotting(x_start, x_goal)
        self.utils = utils.Utils([x_start,x_goal])

        self.fig= plt.subplots()
        self.ax = plt.gca()
        self.delta = self.utils.delta
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_polygon_byzone = self.env.obs_polygon_byzone
        self.obs_polygon_fsb = self.env.obs_polygon_fsb
        self.obs_boundary = self.env.obs_boundary
        # self.crs_roads = self.env.crs_roads
        # self.crs_railways = self.env.crs_railways

        self.V = [self.x_start]
        self.X_soln = set()
        self.path = None
        self.rev_path = None
        self.crossCnt = 0

    def init(self):
        cMin, theta = self.get_distance_and_angle(self.x_start, self.x_goal)
        C = self.RotationToWorldFrame(self.x_start, self.x_goal, cMin)
        xCenter = np.array([[(self.x_start.x + self.x_goal.x) / 2.0],
                            [(self.x_start.y + self.x_goal.y) / 2.0], [0.0]])
        x_best = self.x_start

        return theta, cMin, xCenter, C, x_best

    def planning(self)->bool:
        theta, dist, x_center, C, x_best = self.init()
        c_best = np.inf
        c_best_prev = np.inf
        flag_max_iter = False
        cnt = self.iter_max/20
        for k in range(self.iter_max):
            if self.X_soln:
                cost = {node: self.Cost(node) for node in self.X_soln}
                x_best = min(cost, key=cost.get)
                sup = self.get_distance_and_angle(x_best,self.x_goal)
                c_best = cost[x_best]+sup[0]
                # c_best = cost[x_best]
            x_rand = self.Sample(c_best, dist, x_center, C)
            x_nearest = self.Nearest(self.V, x_rand)
            x_new = self.Steer(x_nearest, x_rand)

            if x_new and not self.utils.is_collision(x_nearest, x_new):
                X_near = self.Near(self.V, x_new)
                c_min = self.Cost(x_nearest) + self.Line(x_nearest, x_new)
                self.V.append(x_new)

                # choose parent
                for x_near in X_near:
                    c_new = self.Cost(x_near) + self.Line(x_near, x_new)
                    if c_new < c_min:
                        x_new.parent = x_near
                        c_min = c_new

                # rewire
                for x_near in X_near:
                    c_near = self.Cost(x_near)
                    c_new = self.Cost(x_new) + self.Line(x_new, x_near)
                    if c_new < c_near:
                        x_near.parent = x_new

                if self.InGoalRegion(x_new):
                    if not self.utils.is_collision(x_new, self.x_goal):
                        self.X_soln.add(x_new)
                        # new_cost = self.Cost(x_new) + self.Line(x_new, self.x_goal)
                        # if new_cost < c_best:
                        #     c_best = new_cost
                        #     x_best = x_new

            if k % 20 == 0:
                self.animation(x_center=x_center, c_best=c_best, dist=dist, theta=theta)

            if(c_best != np.inf):
                cnt=cnt-1
                if(cnt <= 0):
                    print("Reach .1'%' approximation")
                    break
            if(k == self.iter_max-1):
                flag_max_iter = True
                print('Reach maximum iterations')

        self.path = self.ExtractPath(x_best)
        if(flag_max_iter and self.path == [self.x_goal.toPoint(),self.x_start.toPoint()]):
            # df = pds.DataFrame([])
            # df.to_csv("data/path_planned.csv",index=False)
            return False
        self.rev_path = self.path
        self.rev_path.reverse()
        # df = pds.DataFrame(self.rev_path)
        # df.to_csv("data/path_planned.csv",index=False)
        self.animation(x_center=x_center, c_best=c_best, dist=dist, theta=theta)
        plt.plot([x for x, _ in self.path], [y for _, y in self.path], '-r')
        plt.pause(0.01)
        plt.show(block = False)
        return True
    
    def Steer(self, x_start, x_goal):
        dist, theta = self.get_distance_and_angle(x_start, x_goal)
        dist = min(self.step_len, dist)
        node_new = Node((x_start.x + dist * math.cos(theta),
                         x_start.y + dist * math.sin(theta)))
        node_new.parent = x_start

        return node_new

    def Near(self, nodelist, node):
        n = len(nodelist) + 1
        r = 50 * math.sqrt((math.log(n) / n))

        dist_table = [(nd.x - node.x) ** 2 + (nd.y - node.y) ** 2 for nd in nodelist]
        X_near = [nodelist[ind] for ind in range(len(dist_table)) if dist_table[ind] <= r ** 2 and
                  not self.utils.is_collision(nodelist[ind], node)]

        return X_near

    def Sample(self, c_max, c_min, x_center, C):
        if c_max < np.inf:
            # print(c_max,c_min)
            r = [c_max / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0,
                 math.sqrt(c_max ** 2 - c_min ** 2) / 2.0]
            L = np.diag(r)

            while True:
                x_ball = self.SampleUnitBall()
                # print(x_ball)
                x_rand = np.dot(np.dot(C, L), x_ball) + x_center
                if self.x_range[0] + self.delta <= x_rand[0] <= self.x_range[1] - self.delta and \
                        self.y_range[0] + self.delta <= x_rand[1] <= self.y_range[1] - self.delta:
                    break
            x_rand = Node((x_rand[(0, 0)], x_rand[(1, 0)]))
        else:
            x_rand = self.SampleFreeSpace()

        return x_rand

    @staticmethod
    def SampleUnitBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])

    def SampleFreeSpace(self):
        delta = self.delta

        if np.random.random() > self.goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.x_goal

    def ExtractPath(self, node):
        path = [[self.x_goal.x, self.x_goal.y]]

        while node.parent:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.x_start.x, self.x_start.y])

        return path

    def InGoalRegion(self, node):
        if self.Line(node, self.x_goal) < self.step_len:
            return True

        return False

    @staticmethod
    def RotationToWorldFrame(x_start, x_goal, L):
        a1 = np.array([[(x_goal.x - x_start.x) / L],
                       [(x_goal.y - x_start.y) / L], [0.0]])
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T
        U, _, V_T = np.linalg.svd(M, True, True)
        C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T

        return C

    @staticmethod
    def Nearest(nodelist, n):
        return nodelist[int(np.argmin([(nd.x - n.x) ** 2 + (nd.y - n.y) ** 2
                                       for nd in nodelist]))]

    @staticmethod
    def Line(x_start, x_goal):
        return math.hypot(x_goal.x - x_start.x, x_goal.y - x_start.y)

    def Cost(self, node):
        if node == self.x_start:
            return 0.0

        if node.parent is None:
            return np.inf

        cost = 0.0
        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent  
        return cost

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)

    def animation(self, x_center=None, c_best=None, dist=None, theta=None):
        plt.cla()
        self.plot_grid("Informed rrt*, N = " + str(self.iter_max))
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

        for node in self.V:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-g")

        if c_best != np.inf:
            self.draw_ellipse(x_center, c_best, dist, theta)
    
        plt.pause(0.01)

    def plot_grid(self, name):
        self.ax.add_patch(
            patches.Polygon(
                self.obs_boundary.interiors[0].coords,
                edgecolor='blue',
                facecolor='blue',
                fill=False
            )
        )
        for poly in self.obs_polygon_fsb:
            self.ax.add_patch(
                patches.Polygon(
                    poly.exterior.coords,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )
        for poly in self.obs_polygon_byzone:
            self.ax.add_patch(
                patches.Polygon(
                    poly.exterior.coords,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        plt.plot(self.x_start.x, self.x_start.y, "bs", linewidth=3)
        plt.plot(self.x_goal.x, self.x_goal.y, "rs", linewidth=3)

        plt.title(name)
        plt.axis("equal")

    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.1)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, ".b")
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)


def main():

    data = pds.read_csv('data/path_points_2.csv')
    x_start = tuple(data.values[0])  # Starting node
    print ("start: " + str(x_start))
    x_goal = tuple(data.values[1])  # Starting node
    print ("goal: " + str(x_goal))
    step = max(abs(x_start[0]- x_goal[0]),abs(x_start[1]- x_goal[1]))/5.0

    tic = time.perf_counter()   
    rrt_star = IRrtStar(x_start, x_goal, step, 0.10, 12, 1000)
    print("Data Filtering execution time: " + str(time.perf_counter()-tic))
    
    tic = time.perf_counter()
    if not rrt_star.planning():
        print("Cannot plan path")
    print("Planning algorithm execution time: " + str(time.perf_counter()-tic))
    

if __name__ == '__main__':
    main()
