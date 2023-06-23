import pandas as pd
import numpy as np
import math
def generate_2_points():
    ## generate 2 points in test areas
    data = pd.read_csv("data/test_area.csv")
    df = data[['long','lat']]
    polygon = df.to_numpy()
    print(polygon)
    def point_in_polygon(point, polygon):
        x, y = point
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(n+1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def generate_random_points_in_polygon(polygon, num_points):
        min_x, min_y = np.min(polygon, axis=0)
        max_x, max_y = np.max(polygon, axis=0)
        points = []
        while len(points) < num_points:
            random_point = np.random.uniform(min_x, max_x), np.random.uniform(min_y, max_y)
            if point_in_polygon(random_point, polygon):
                if(not len(points)):
                    points.append(random_point)
                elif(math.dist(random_point,points[-1])>=0.01):
                    points.append(random_point)                   
        return points

    num_points = 2  # Number of random points to generate
    path_points = generate_random_points_in_polygon(polygon, num_points)
    df = pd.DataFrame(path_points)
    df.to_csv("data/path_points.csv",index=False)


