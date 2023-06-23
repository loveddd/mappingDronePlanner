import os
import shapely
import geopandas as gpd
import pandas as pds
import matplotlib.pyplot as plt

class DataFilter:
    def __init__(self,
                 path_to_fsb_data,
                 path_to_byzone_data,
                 path_points:list,
                 path_to_roads_data,
                 path_to_railways_data) -> None:
        
        self.path_fsb = path_to_fsb_data
        self.path_byzone = path_to_byzone_data
        self.path_points = path_points
        self.path_roads = path_to_roads_data
        self.path_railways = path_to_railways_data

        self.rect = None
        self.bc = 0.01  #longitude
        self.bounds = shapely.LineString(path_points).bounds
        self.boundary = self.getBoundary(self)
        if path_to_fsb_data :
            self.fsb = self.getFSB(self)
        if path_to_byzone_data:
            self.byzone = self.getByzone(self)
        if path_to_roads_data:
            self.roads = self.getRoads(self)
        if path_to_railways_data:
            self.railways = self.getRailways(self)


    @staticmethod
    def getBoundary(self)-> gpd.GeoSeries:
        bds = self.bounds
        rect = [(bds[0]-self.bc,bds[1]-self.bc),(bds[0]-self.bc,bds[3]+self.bc),
                (bds[2]+self.bc,bds[3]+self.bc),(bds[2]+self.bc,bds[1]-self.bc)]
        self.rect = rect
        rec_series = gpd.GeoSeries(shapely.Polygon(rect),crs="EPSG:4326")
        return rec_series
    
    @staticmethod    
    def getFSB(self)-> gpd.GeoDataFrame:
        fsb = gpd.read_file(self.path_fsb,bbox=self.boundary.to_crs(crs="EPSG:25832"))
        fsb = fsb.to_crs(crs="EPSG:3857")
        fsb = fsb.buffer(30).to_frame() #buffer 30 meter
        fsb = fsb.dissolve().to_crs(crs="EPSG:4326")
        fsb = fsb.clip(self.boundary)
        return fsb
    
    @staticmethod
    def getByzone(self)-> gpd.GeoDataFrame:
        byzone = gpd.read_file(self.path_byzone,bbox=self.boundary.to_crs(crs="EPSG:25832"))
        byzone = byzone.to_crs(crs="EPSG:3857")
        byzone = byzone.buffer(200).to_frame() #buffer 180 meter
        byzone = byzone.dissolve().to_crs(crs="EPSG:4326")
        byzone = byzone.clip(self.boundary)
        return byzone   

    @staticmethod
    def getRoads(self):
        roads = gpd.read_file(self.path_roads,bbox=self.boundary)
        roads = roads.clip(self.boundary)
        return roads
    
    @staticmethod
    def getRailways(self):
        railways = gpd.read_file(self.path_railways,bbox=self.boundary)
        railways = railways.clip(self.boundary)
        return railways



if __name__ == "__main__":

    path_byzone = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/byzone/byzone_s/pdk_zonekort_wfs.shp"
    path_fsb = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_buildings_a_free_1.shp"
    path_road = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_roads_free_1.shp"
    path_rail = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_railways_free_1.shp"
    path_to_pathpoint_file = "data/path_points_2.csv"
    path_points = pds.read_csv(path_to_pathpoint_file).values
    fltr = DataFilter(path_fsb,
                      path_byzone,
                      path_points,
                      path_road,
                      path_rail)

    fig,ax = plt.subplots()
    fltr.fsb.plot(ax=ax)
    fltr.byzone.plot(ax=ax)
    fltr.roads.plot(ax=ax)
    plt.pause(0.01)
    plt.show(block = True)
        
