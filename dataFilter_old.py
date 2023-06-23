import sys,os
import pandas
from qgis.gui import (
QgsLayerTreeMapCanvasBridge,
)
from PyQt5 import *
from qgis.core import *
from qgis import processing
from qgis.PyQt.QtCore import QVariant

# Supply path to qgis install location
## paths and data
buffer_coefficient = 0.003  #longitude
path_to_150mbyzone_layer = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/byzone/150meter_byzone.shp"
path_to_180mbyzone_layer = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/byzone/180meter_byzone.shp"
path_to_fsb_layer = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/modified-data/FSB/FSB-30.shp"
path_to_road_layer = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_roads_free_1.shp"
path_to_rail_layer = os.path.dirname(os.path.abspath(__file__))+"/../gis_data/denmark-latest-free/gis_osm_railways_free_1.shp"
path_to_pathpoint_file = "data/path_points.csv"

class DataFilter:
    def __init__(self) -> None:
        pass

    def filter(self,path_points):
        QgsApplication.setPrefixPath("/usr", True)
        QgsApplication.setPluginPath("/usr/lib/qgis/plugins")
        # Create a reference to the QgsApplication.  Setting the
        # second argument to False disables the GUI.
        qgs = QgsApplication([], False)
        # Load providers
        qgs.initQgis()

        sys.path.append('/usr/lib/qgis')
        sys.path.append('/usr/share/qgis/python/plugins')
        from processing.core.Processing import Processing
        Processing.initialize()
        import processing as ps

        
        ## load path points
        # path_points = pandas.read_csv(self.path_to_pathpoint_file).values
        print(path_points)

        project = QgsProject.instance()
        project.setCrs(QgsCoordinateReferenceSystem.fromEpsgId(4326),False)

        ## generate shp file for path data
        fields = QgsFields()
        fields.append(QgsField("first", QVariant.Int))
        fields.append(QgsField("second", QVariant.String))

        crs = project.crs()
        transform_context = QgsProject.instance().transformContext()
        save_options = QgsVectorFileWriter.SaveVectorOptions()
        save_options.driverName = "ESRI Shapefile"
        save_options.fileEncoding = "UTF-8"

        writer = QgsVectorFileWriter.create(
        "data/temp/path_points.shp",
        fields,
        QgsWkbTypes.Type.MultiPoint,
        crs,
        transform_context,
        save_options
        )

        if writer.hasError() != QgsVectorFileWriter.NoError:
            print("Error when creating shapefile: ",  writer.errorMessage())

        fet = QgsFeature()
        points = [QgsPointXY(*point) for point in path_points]
        fet.setGeometry(QgsGeometry.fromMultiPointXY(points))
        fet.setAttributes([1, "text"])
        writer.addFeature(fet)

        del writer


        ## get bounding box of path area
        multipts = QgsMultiPoint()
        for point in path_points:
            multipts.addGeometry(QgsPoint(*point))
        boundbx = multipts.boundingBox()
        boundbx_bufferred = boundbx.buffered(buffer_coefficient)
        ## generate shp file for boundbx
        fields = QgsFields()
        fields.append(QgsField("first", QVariant.Int))
        fields.append(QgsField("second", QVariant.String))

        crs = project.crs()
        transform_context = QgsProject.instance().transformContext()
        save_options = QgsVectorFileWriter.SaveVectorOptions()
        save_options.driverName = "ESRI Shapefile"
        save_options.fileEncoding = "UTF-8"

        writer = QgsVectorFileWriter.create(
        "data/temp/plan_area.shp",
        fields,
        QgsWkbTypes.Type.Polygon,
        crs,
        transform_context,
        save_options
        )

        if writer.hasError() != QgsVectorFileWriter.NoError:
            print("Error when creating shapefile: ",  writer.errorMessage())

        fet = QgsFeature()
        fet.setGeometry(QgsGeometry.fromRect(boundbx_bufferred))
        fet.setAttributes([1, "text"])
        writer.addFeature(fet)

        del writer

        ## search for algorithmn
        for alg in qgs.processingRegistry().algorithms():
            if("reproject" in alg.id()):
                # print(alg.id(), "->", alg.displayName())
                pass

        ## extract boundary
        def vertices_to_csv(input:str,output:str)->bool:
            fsb = []
            vl = QgsVectorLayer(input, "layer", "ogr")
            if vl.isValid():
                for feature in vl.getFeatures():
                    geometry = feature.geometry()
                    longitude = geometry.asPoint().x()
                    latitude = geometry.asPoint().y()
                    fsb.append([longitude,latitude])
                project.addMapLayer(vl)
                df = pandas.DataFrame(fsb,columns=['Longitude','Latitude'])
                df.to_csv(output,index=False)
            return True

        output = ps.run("native:extractvertices",{
            'INPUT':"data/temp/plan_area.shp",
            'OUTPUT':'data/temp/boundary_verctices.shp'
        })

        if not vertices_to_csv('data/temp/boundary_verctices.shp','data/obstacles/boundary.csv'):
            print("vertices_to_csv fault")

        ## extract fsb data
        feedback = QgsProcessingFeedback()
        def clip_area(input:str,output:str,overlay:str):
            params = {
                'INPUT': input,
                'OUTPUT': output,
                'OVERLAY': overlay 
            }   
            return ps.run("native:clip",params,feedback=feedback)

        def dissolve_area(input:str,output:str,field:str):
            params = {
                'FIELD' : field, 
                'INPUT' : input, 
                'OUTPUT' : output
            }   
            return ps.run("native:dissolve",params,feedback=feedback)

        clip_area(path_to_fsb_layer,'data/temp/fbs_clipped.shp',"data/temp/plan_area.shp")
        output = dissolve_area('data/temp/fbs_clipped.shp','data/temp/fbs_dissolved.shp',[])

        ps.run("native:extractvertices",{
            'INPUT':'data/temp/fbs_dissolved.shp',
            'OUTPUT':'data/temp/fsb_verctices.shp'
        })

        ps.run("native:reprojectlayer",{
            'INPUT' : 'data/temp/fsb_verctices.shp', 
            'OPERATION' : '+proj=pipeline +step +inv +proj=utm +zone=32 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad +xy_out=deg', 
            'OUTPUT' : 'data/temp/fsb_reproject.shp', 
            'TARGET_CRS' : QgsCoordinateReferenceSystem('EPSG:4326') 
        })

        if not vertices_to_csv('data/temp/fsb_reproject.shp','data/obstacles/fsb.csv'):
            print("vertices_to_csv fault")

        ## extract byzone data
        clip_area(path_to_180mbyzone_layer,'data/temp/180_clipped.shp',"data/temp/plan_area.shp")
        ps.run("native:extractvertices",{
            'INPUT':'data/temp/180_clipped.shp',
            'OUTPUT':'data/temp/180_verctices.shp'
        })
        ps.run("native:reprojectlayer",{
            'INPUT' : 'data/temp/180_verctices.shp', 
            'OPERATION' : '+proj=pipeline +step +inv +proj=utm +zone=32 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad +xy_out=deg', 
            'OUTPUT' : 'data/temp/180_reprojected.shp', 
            'TARGET_CRS' : QgsCoordinateReferenceSystem('EPSG:4326') 
        })

        if not vertices_to_csv('data/temp/180_reprojected.shp','data/obstacles/180.csv'):
            print("vertices_to_csv fault")

        clip_area(path_to_150mbyzone_layer,'data/temp/150_clipped.shp',"data/temp/plan_area.shp")
        ps.run("native:extractvertices",{
            'INPUT':'data/temp/150_clipped.shp',
            'OUTPUT':'data/temp/150_verctices.shp'
        })
        ps.run("native:reprojectlayer",{
            'INPUT' : 'data/temp/150_verctices.shp', 
            'OPERATION' : '+proj=pipeline +step +inv +proj=utm +zone=32 +ellps=GRS80 +step +proj=unitconvert +xy_in=rad +xy_out=deg', 
            'OUTPUT' : 'data/temp/150_reprojected.shp', 
            'TARGET_CRS' : QgsCoordinateReferenceSystem('EPSG:4326') 
        })
        if not vertices_to_csv('data/temp/150_reprojected.shp','data/obstacles/150.csv'):
            print("vertices_to_csv fault")

        #clip roads and rails
        clip_area(path_to_road_layer,'data/obstacles/roads_clipped.shp',"data/temp/plan_area.shp")
        clip_area(path_to_rail_layer,'data/obstacles/rails_clipped.shp',"data/temp/plan_area.shp")

        # Finally, exitQgis() is called to remove the
        # provider and layer registries from memory
        print("End of Filter")
        qgs.exitQgis()

if __name__ == '__main__':
    import generate2points
    # generate2points.generate_2_points()
    df = DataFilter()
    path_points = pandas.read_csv("data/path_points_2.csv").values
    df.filter(path_points)
