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

# Gives you the index for each field for a given layer
def fields(ds,LayerName):
    layer = ds.GetLayerByName(LayerName)
    layer_def = layer.GetLayerDefn()
#    feat = layer.GetNextFeature()
    print str(LayerName)
    for i in range(layer_def.GetFieldCount()):
        print str(i) + ': ' + layer_def.GetFieldDefn(i).GetName()

# Converts the number stored in Category of Lights to string    
def category_lights(feat):
    index = str(feat.GetField(12))
    if index == '1':
        return 'Directional Function'
#        #Index 2 and 3 are not used
#    elif index =='2':
#        return ''
#    elif index =='3':
#        return ''
    elif index =='4':
        return 'Leading'
    elif index =='5':
        return 'Aero'
    elif index =='6':
        return 'Air Obstruction'
    elif index == '7':
        return 'Fog Detector'
    elif index == '8':
        return 'Flood'
    elif index =='9':
        return 'Strip'
    if index == '10':
        return 'Subsidiary'
    if index == '11':
        return 'Spot'
    if index == '12':
        return 'Front'
    if index == '13':
        return 'Rear'
    if index == '14':
        return 'Lower'
    if index == '15':
        return 'Upper'
    if index == '16':
        return 'Moire Effect'
    if index == '17':
        return 'Emergency'
    if index == '18':
        return 'Bearing'
    if index == '19':
        return 'Horizontally Disposed'
    elif index =='20':
        return 'Vertically Disposed'
    else:
        return 'Marine'

# Converts the number stored in category of Landmark or Silo/Tank to a string
def category_landmark(feat, name):
    if name == 'LNDMRK':
        index = str(feat.GetField(11))
        if index == '1':
            return 'Cairn'
        elif index =='2':
            return 'Cemetery'
        elif index =='3':
            return 'Chimney'
        elif index =='4':
            return 'Dish Aerial'
        elif index =='5':
            return 'Flagstaff'
        elif index =='6':
            return 'Flare Stack'
        elif index == '7':
            return 'Mast'
        elif index == '8':
            return 'Windsock'
        elif index =='9':
            return 'Monument'
        elif index == '10':
            return 'Column'
        elif index == '11':
            return 'Memorial Plaque'
        elif index == '12':
            return 'Obelisk'
        elif index == '13':
            return 'Statue'
        elif index == '14':
            return 'Cross'
        elif index == '15':
            return 'Dome'
        elif index == '16':
            return 'Radar Scanner'
        elif index == '17':
            return 'Tower'
        elif index == '18':
            return 'Windmill'
        elif index == '19':
            return 'Windmotor'
        elif index =='20':
            return 'Spire'
        elif index =='21':
            return 'Large On Land Rock'
        else:
            return 'Unknown Landmark'
    elif name == 'SILTNK':
        index = str(feat.GetField(12))
        if index == '1':
            return 'Silo'
        elif index =='2':
            return 'Tank'
        elif index =='3':
            return 'Grain Elevator'
        elif index =='4':
            return 'Water Tower'
        else:
            return 'Unknown'

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
        if WL != 0 or LayerName == 'DEPCNT':
            t_lvl = calc_t_lvl(depth, WL)
        # If it is Land set threat level to 5
        elif LayerName == 'LNDARE' or LayerName == 'DYKCON' or LayerName == 'PONTON' or LayerName == 'COALNE':
            t_lvl = 5
        elif LayerName == 'LIGHTS':
            t_lvl = -2
#            print category_lights(feat)
#            print str(geom.GetX())+', '+str(geom.GetY())
#            print feat.GetField(35)
        # If it is a Buoy, Light or Beacon set threat level to 3
        elif LayerName == 'BOYISD' or LayerName == 'BOYSPP' or LayerName == 'BOYSAW' or LayerName == 'BOYLAT' or LayerName == 'BCNSPP' or LayerName == 'BCNLAT':
            t_lvl = 3
        elif LayerName == 'LNDMRK':
            t_lvl = -1
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
        if LayerName == 'LNDMRK':
            c = category_landmark(feat,LayerName)
            if c is not None and feat.GetField(16) is not None:
                print 'Cat: ' + c + ', Vis: '+ str(2-feat.GetField(16))
            new_feat.SetField('Cat', c) # Category - store as string and not a number
            new_feat.SetField('Visual', 2-feat.GetField(16)) # Visually Conspicuous (Y - store as 1, N - store as 0)
        if LayerName == 'SILTNK':
            c = category_landmark(feat,LayerName)
            if c is not None and feat.GetField(17) is not None:
                print 'Cat: ' + c + ', Vis: '+ str(2-feat.GetField(17))
            new_feat.SetField('Cat', c) # Category - store as string and not a #
            new_feat.SetField('Visual', 2-feat.GetField(17)) # Visually Conspicuous (Y - store as 1, N - store as 0)
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
## Landmark
#fields(ds, 'LNDMRK') # 11 - Category, 12 - Color, 13 - Color Patern, 16 - Conspicuous
#fields(ds, 'SILTNK') # 12 - Category, 13 - Color, 14 - Color Patern, 17 - Conspicuous

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
layer_pnt.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
layer_pnt.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
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
layer_poly.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
layer_poly.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
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
layer_line.CreateField(ogr.FieldDefn('Cat', ogr.OFTString))
layer_line.CreateField(ogr.FieldDefn('Visual', ogr.OFTInteger))
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
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'LNDMRK', layer_pnt, layer_line, layer_poly)
layer_pnt, layer_line, layer_poly = enc2shp(ds, 'SILTNK', layer_pnt, layer_line, layer_poly)

# Close and save the files
ds = ds_pnt = ds_line = ds_poly = None
