# -*- coding: utf-8 -*-
"""
Created on Thu May 26 14:14:51 2016

@author: mapper
"""

from osgeo import ogr, gdal
from os import path # Used to check if the file already exists

# This function uses the water level and depth attributes for a feature and 
#   calculates the threat level for that obstacle.
def calc_t_lvl(depth, WL):
    # Obstacle Partly submerged at high water, Always Dry, Awash, Floating, or 0<=Z<=3
    if (WL == 1 or WL == 2 or WL == 5 or WL == 7 or depth <= 0):
        return 4
    # Obstacle Covers and uncovers, Subject to inundation or flooding, or 0<Z<3
    elif (WL == 4 or WL == 6 or depth < 1.5):
        return 3
    # Obstacle is alway below surface
    elif (WL == 3):
        # 3<=Z<6 or depth is unknown (9999)
        if (depth < 3 or depth == 9999):
            return 2
        # 6<=Z<10
        elif (depth >=3 and depth <= 5):
            return 1
        # Z > 10
        else:
            return 0

# Layers that are Multipoints --> SOUNDG
def LayerMultiPoint (ds, LayerName_mp, out_layer):
    layer_mp = ds.GetLayerByName(LayerName_mp)
    feat_mp = layer_mp.GetNextFeature()
    
    while feat_mp is not None:
        multi_geom = feat_mp.GetGeometryRef()
        for iPnt in range(multi_geom.GetGeometryCount()):
            # Create point
            point = ogr.Geometry(ogr.wkbPoint)
            pnt = multi_geom.GetGeometryRef(iPnt)
            point.SetPoint_2D(0,pnt.GetX(),pnt.GetY())

            # Create a new feature (attribute and geometry)
            defn_pnt = out_layer.GetLayerDefn()
            new_feat = ogr.Feature(defn_pnt)
            new_feat.SetField('T_lvl', calc_t_lvl(pnt.GetZ(),0))
            new_feat.SetField('Type', LayerName_mp)
    
            # Make a geometry, from Shapely object
            new_feat.SetGeometry(point)

            out_layer.CreateFeature(new_feat) 
        feat_mp = layer_mp.GetNextFeature()
    
    return layer_pnt

# Converts the ENC File to 3 different shape files    
def enc2shp(ds, LayerName, output_pnt, output_line, output_poly):
    layer = ds.GetLayerByName(LayerName)
    layer_def = layer.GetLayerDefn() # needed to find the name of the fields
    feat = layer.GetNextFeature()
    
    while feat is not None:
        geom = feat.GetGeometryRef() # Needed for determining the geom type and to export the object to wkt
        geo_name = geom.GetGeometryName()    
        
        # Export this feature to WTK for easy conversion to Shapefile
        wkt = geom.ExportToWkt()
    
        ## Initialize WL, depth, and threat level
        WL = 0
        depth = 9999
        t_lvl = 0
    
        ## Cycle through the fields and store the ones that are useful
        for i in range(feat.GetFieldCount()):
            name = layer_def.GetFieldDefn(i).GetName()
            if name == 'WATLEV':
                WL = feat.GetField(i)
            elif name == 'VALSOU':
                depth = feat.GetField(i)
            elif name == 'VALDCO': # value of the depth contour
                depth = feat.GetField(i)
                
    
        ## Update Threat Level
        # If WL has been updated use the function calc_t_lvl to calculate threat level
        if WL != 0:
            t_lvl = calc_t_lvl(depth, WL)
        # If it is Land set threat level to 5
        elif LayerName == 'LNDARE' or LayerName == 'DYKCON' or LayerName == 'PONTON' or LayerName == 'COALNE':
            t_lvl = 5
        # If it is a Buoy, Light or Beacon set threat level to 4
        elif LayerName == 'LIGHTS' or LayerName == 'BOYISD' or LayerName == 'BOYSPP' or LayerName == 'BOYSAW' or LayerName == 'BOYLAT' or LayerName == 'BCNSPP' or LayerName == 'BCNLAT':
            t_lvl = 3
#        print t_lvl
        out_layer = None
        # Choose the correct layer for 
        if geo_name == 'POINT':
            out_layer = output_pnt
            obj = ogr.Geometry(ogr.wkbPoint)
        elif geo_name == 'POLYGON':
            out_layer = output_poly
            obj = ogr.Geometry(ogr.wkbPolygon)
        elif geo_name == 'LINESTRING':
            out_layer = output_line
            obj = ogr.Geometry(ogr.wkbLineString)
            
        # Create a new feature (attribute and geometry)
        defn_pnt = out_layer.GetLayerDefn()
        new_feat = ogr.Feature(defn_pnt)
        new_feat.SetField('T_lvl', t_lvl)
        new_feat.SetField('Type', LayerName)
    
        # Make a geometry from wkt object
        obj = ogr.CreateGeometryFromWkt(wkt)
        new_feat.SetGeometry(obj)

        # Output the new feature to the correct output shapefile
        out_layer.CreateFeature(new_feat)
        
        # Get the next feature
        feat = layer.GetNextFeature()
        
        # Save the new layer
        if geo_name == 'POINT':
            output_pnt = out_layer 
            
        elif geo_name == 'POLYGON':
            output_poly = out_layer
            
        elif geo_name == 'LINESTRING':
            output_line = out_layer 

    return output_pnt, output_line, output_poly
    
##---------------------------------------------------------------------------##
### Points
## Underwater Rocks   
#fields(ds, 'UWTROC') # 20 - Value of sounding, 22 - WL, 31 - Source Date
# Isolated Danger Buoy
#fields(ds, 'BOYISD') # 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
## Lateral Buoy
#fields(ds, 'BOYLAT') # 11 - Buoy Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern 
## Special Purpose Buoy
#fields(ds, 'BOYSPP') # 11 - Buoy Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern
## Safe Water Buoy
#fields(ds, 'BOYSAW') # 11 - Buoy Shape, 12 - Color, 13 - Color Pattern 
## Lateral Beacon
#fields(ds, 'BCNLAT') # 11 - Beacon Shape, 12 - Category of Lateral Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
## Special Purpose Beacon
#fields(ds, 'BCNSPP') # 11 - Beacon Shape, 12 - Category of Special Purpose Mark, 13 - Color, 14 - Color Pattern, 20 - Elevation, 21 - Height
# Lights
#fields(ds, 'LIGHTS') # 11- Category of Light, 16 - Height, 23 - Light Orientation

### Multipoint
## Soundings
#fields(ds, 'SOUNDG') # 17 - Vertical Datum, 27 - Source Date

### Polygons
## DRYDOC

### Multiple Types - WRECKS, OBSTRN, LNDARE, DEPARE, PONTON, DEPCNT, DYKCON
## Wrecks       
#fields(ds, 'WRECKS') # 11 - Catagory of wreck, 22 - Value of the Sounding, 26 - WL, 35 - Source Date
## Obstruction
#fields(ds, 'OBSTRN') # 11 - Category of Obstruction, 24 - Value of sounding, 28 - WL, 38 - Source Date
## Land Area
#fields(ds, 'LNDARE')
## Depth Area
#fields(ds, 'DEPARE') # 11 - Minimum depth in range, 12 - Maximum depth in range
##Pontoon
#fields(ds, 'PONTON') # 23 - Vertical Length
## Depth Contour
#fields(ds, 'DEPCNT') # 11 - Depth of the contour
## Dyke
#fields(ds, 'DYKCON') # 19 - Vertical Length

##---------------------------------------------------------------------------##
# Path to the ENC and output shapefiles
s57filename =  "/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/US5NH02M.000"
outfile_pnt =  '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_pnt2.shp'
outfile_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'
outfile_line = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_line2.shp'

# Enable GDAL/OGR exceptions
gdal.UseExceptions()

# Open S57 File
ds = ogr.Open( s57filename )

##---------------------------------------------------------------------------##
## Create the Shapefile for points
# Use the Shapefile driver for gdal
driver = ogr.GetDriverByName('Esri Shapefile')

# If there already is a file with that name, delete it
if path.exists(outfile_pnt):
    driver.DeleteDataSource(outfile_pnt)

# Open the point shapefile
ds_pnt = driver.CreateDataSource(outfile_pnt)
layer_pnt = ds_pnt.CreateLayer('ENC', None, ogr.wkbPoint)

# Add the Threat Level and Type Attributes
layer_pnt.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
layer_pnt.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
defn_pnt = layer_pnt.GetLayerDefn()

##---------------------------------------------------------------------------##
## Create the Shapefile for Polygons
# If there already is a file with that name, delete it
if path.exists(outfile_poly):
    driver.DeleteDataSource(outfile_poly)

# Open the polygon shapefile
ds_poly = driver.CreateDataSource(outfile_poly)
layer_poly = ds_poly.CreateLayer('ENC', None, ogr.wkbPolygon)

# Add the Threat Level and Type Attributes
layer_poly.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
layer_poly.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
defn_poly = layer_poly.GetLayerDefn()

##---------------------------------------------------------------------------##
## Create the Shapefile for Line
# If there already is a file with that name, delete it
if path.exists(outfile_line):
    driver.DeleteDataSource(outfile_line)

# Open the Line shapefile
ds_line = driver.CreateDataSource(outfile_line)
layer_line = ds_line.CreateLayer('ENC', None, ogr.wkbLineString)

# Add the Threat Level and Type Attributes
layer_line.CreateField(ogr.FieldDefn('T_lvl', ogr.OFTInteger))
layer_line.CreateField(ogr.FieldDefn('Type', ogr.OFTString))
defn_line = layer_line.GetLayerDefn()

##---------------------------------------------------------------------------##
# Layers that are only points --> UWTROC, LIGHTS, BOYSPP, BOYISD, BOYSAW, BOYLAT, BCNSPP, BCNLAT
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'UWTROC', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'LIGHTS', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BOYSPP', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BOYISD', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BOYSAW', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BOYLAT', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BCNSPP', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'BCNLAT', layer_pnt, layer_line, layer_poly)

# Layers that are Multipoints --> SOUNDG
layer_pnt = LayerMultiPoint (ds, 'SOUNDG', layer_pnt)

##---------------------------------------------------------------------------##
# Layers that have multiple types --> WRECKS, OBSTRN, LNDARE, DEPARE, PONTON, DEPCNT, DYKCON
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'WRECKS', layer_pnt, layer_line, layer_poly)
#layer_pnt, layer_line, layer_poly = enc2shp(ds, 'OBSTRN', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'LNDARE', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'PONTON', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'DEPCNT', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'DYKCON', layer_pnt, layer_line, layer_poly)

# Close and save the files
ds = ds_pnt = ds_line = ds_poly = None
