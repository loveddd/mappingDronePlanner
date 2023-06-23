import pandas as pds
import os
import geopandas as gpd
import shapely
import dataFilter as df
import numpy as np

path_byzone = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/byzone/byzone_s/pdk_zonekort_wfs.shp"
path_fsb = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_buildings_a_free_1.shp"
path_road = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_roads_free_1.shp"
path_rail = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_railways_free_1.shp"
path_to_pathpoint_file = "data/path_points_2.csv"

class Env:
    
    def __init__(self,path_points): 
        self.fltr = df.DataFilter(path_fsb,
                    path_byzone,
                    path_points,
                    [],
                    [])       
        self.x_range = (self.fltr.rect[0][0],self.fltr.rect[2][0])
        self.y_range = (self.fltr.rect[0][1],self.fltr.rect[1][1])
        self.obs_boundary = self.obs_boundary(self)
        self.obs_polygon_fsb = self.obs_polygon_fsb(self)
        self.obs_polygon_byzone = self.obs_polygon_byzone(self)
        # self.crs_roads = self.crs_roads(self)
        # self.crs_railways = self.crs_railways(self)

    @staticmethod
    def crs_roads(self):
        lst = []
        for road in self.fltr.roads.geometry.values:
            lst.append(road)
        return lst

    @staticmethod
    def crs_railways(self):
        lst = []
        for railway in self.fltr.railways.geometry.values:
            lst.append(railway)
        return lst        


    @staticmethod
    def obs_boundary(self):
        boundary = self.fltr.boundary.exterior.to_list()[0]
        obs_boundary_hole = shapely.Polygon(boundary.coords[::-1])
        obs_boundary = shapely.buffer(shapely.Polygon(boundary),0.0001,cap_style="square",join_style="mitre")
        return shapely.Polygon(obs_boundary.exterior.coords,[obs_boundary_hole.exterior.coords])

    @staticmethod
    def obs_polygon_fsb(self):
        polygons = []
        if not len(self.fltr.fsb.values):
            pass
        else:
            plys = self.fltr.fsb.values[0][0]
            if(type(plys) == shapely.Polygon):
                polygons.append(plys)
            else:            
                for poly in plys.geoms:
                    # print (poly)
                    polygon = shapely.Polygon(poly)
                    if polygon.is_valid:
                        polygons.append(polygon)
        return polygons
    
    @staticmethod
    def obs_polygon_byzone(self):
        polygons = []
        if not len(self.fltr.byzone.values):
            pass
        else:
            plys = self.fltr.byzone.values[0][0]
            if(type(plys) == shapely.Polygon):
                polygons.append(plys)
            else:
                for poly in plys.geoms:
                    # print (poly)
                    polygon = shapely.Polygon(poly)
                    if polygon.is_valid:
                        polygons.append(polygon)
        return polygons

if __name__ == '__main__':
    path_points = pds.read_csv(path_to_pathpoint_file).values
    env = Env(path_points)