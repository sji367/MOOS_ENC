# -*- coding: utf-8 -*-
"""
Created on Fri Apr 15 13:14:36 2016

@author: mapper
"""
# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# MOOS libraries
import pymoos
import time

#=============================================================================#
# This code determines if each point along a line is within a polygon and if it
#   is it outputs that line. If the line intersects the polygon multiple times,
#   then the points between the two intersections are also considered within 
#   the polygon so that it doesn't connect two points that should not be
#   connected.
#=============================================================================#
def poly_line_intersect(poly, line):
    # Create the new line
    new_line = ogr.Geometry(ogr.wkbLineString)
    outside = ogr.Geometry(ogr.wkbLineString)
    pnt = ogr.Geometry(ogr.wkbPoint)
    
    # Temp variables for if there are more than one intersection
    out = False
    first_inside = False
    
    # Cycle through the points along the line to determine if they fall within
    #   the polygon
    for i in range(line.GetPointCount()):
        x,y,z = line.GetPoint(i)
        pnt.AddPoint(x,y)
        # If the point is within the polygon save it
        if pnt.Within(poly):
            # Variable that states if you had the first point inside or not
            first_inside = True 
            # If the last point was inside the polygon, save it without other 
            #   processing            
            if out == False: 
                new_line.AddPoint(x,y)
            # If the previous point was outside the polygon then add the rest 
            #   of the original line to the new line
            else: 
                for ii in range(outside.GetPointCount()):
                    x,y,z = outside.GetPoint(ii)
                    new_line.AddPoint(x,y)
                outside = ogr.Geometry(ogr.wkbLineString)
                out = False
        # If the  point is not within the polygon then add to a line describing
        #   the line outside of the polygon in case it intersects more than 
        #   once
        else: 
            if out == False:
                out = True
            if first_inside == True:
                outside.AddPoint(x,y)
    return new_line
    
#=============================================================================#
# This program uses the information from the ENC Database and prints out the 
#   obstacles onto the pMarineViewer. The obstacles are printed as points with
#   the different colors relating to the threat level of the obstacle.
#=============================================================================#
comms = pymoos.comms()

def on_connect():
    comms.register('VIEW_POLYGON',0)
    comms.register('VIEW_POINT',0)
    return True

def main():
    t = pymoos.time()
    # Calculate the origin 
    p = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    LatOrigin  = 43.071959194444446
    LongOrigin = -70.711610833333339 
    x_origin,y_origin = p(LongOrigin, LatOrigin)
    
    comms.set_on_connect_callback(on_connect);
    comms.run('localhost',9000,'print')
    
    # Start by creating the baseline for the search area polygon 
    ring = ogr.Geometry(ogr.wkbLinearRing)
    poly_filter = ogr.Geometry(ogr.wkbPolygon)

    N_lat = 43.07511878
    E_long = -70.68395689
    S_lat = 43.05780589
    W_long = -70.72434189
    
    # Build a ring of the points fot the search area polygon
    ring.AddPoint(W_long, N_lat)
    ring.AddPoint(E_long, N_lat)
    ring.AddPoint(E_long, S_lat)
    ring.AddPoint(W_long, S_lat)
    ring.AddPoint(W_long, N_lat)
    poly_filter.AddGeometry(ring) # Add the ring to the previously created polygon
     
    
    #easily mark all of the output and input files
    file_pnt = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_pnt2.shp'
    file_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'
    file_line = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_line2.shp'
    
##---------------------------------------------------------------------------##    
    ## Print the point obstacles
    # Get the driver and open the point file
    driver = ogr.GetDriverByName('ESRI Shapefile')
    ds = driver.Open(file_pnt, 0)
    
    # There is only one layer in each file and we are just opening it to see how
    #   features there are in the layer
    layer = ds.GetLayer()
    #print 'Num of Features in File: %d' %layer.GetFeatureCount()
    
    # Filter the layer to only include features with Threat level > 0
    layer.SetSpatialFilter(poly_filter) 
    #layer.SetAttributeFilter("T_lvl != '0'")
    #print 'Num of Features in Filter: %d' %layer.GetFeatureCount()
    
    feature = layer.GetNextFeature()
    cnt = 1
    while feature:    
        geom = feature.GetGeometryRef()
        t_lvl = feature.GetField(0) # Get the Threat Level for that feature
        obs_type = feature.GetField(1) # Get the type of obstacle that the feature is
        time.sleep(.01) # Don't make it faster or it wont print all of the points
        
        # Create the VIEW_MARKER string
        #cntr = cntr+1
        x,y = p(geom.GetX(), geom.GetY())
        new_x = x-x_origin
        new_y = y-y_origin
        location = 'x='+str(new_x)+',y='+str(new_y)+','
        label = 'label='#+obs_type# +' t_lvl: '+ str(t_lvl)
     
        # Change the Color of the point based on the Threat Level
        if t_lvl == 5:
            color = 'vertex_color=black,'
        elif t_lvl == 4:
            #size = 'vertex_size=12,'
            color = 'vertex_color=red,'
        elif t_lvl == 3:
            #size = 'vertex_size=9,'
            color = 'vertex_color=orange,'
        elif t_lvl == 2:
            #size = 'vertex_size=7,'
            color = 'vertex_color=yellow,'
        elif t_lvl == 1:
            #size = 'vertex_size=5,'
            color = 'vertex_color=blue,'
        elif t_lvl == 0:
            #size = 'vertex_size=3,'
            color = 'vertex_color=green,'
        size =  'vertex_size=10,'   
        m = location+size+color#+label+' ' + str(cnt)
        cnt = 1+cnt
        
        # Print the point
        comms.notify('VIEW_POINT', m, pymoos.time()+t)
#        print pymoos.time()-t
        feature = layer.GetNextFeature()
        
##---------------------------------------------------------------------------##
    ## Print the Polygons
    ds = driver.Open(file_poly, 0)
    
    # There is only one layer in each file and we want to open it
    layer = ds.GetLayer()
    
    # Filter the layer to only include features within the area in 
    #   pMarnineViewer
    layer.SetSpatialFilter(poly_filter)
    feature = layer.GetNextFeature()
    while feature:
        geom = feature.GetGeometryRef() # Polygon from shapefile
        # Get the interesection of the polygon from the shapefile and the
        #   outline of tiff from pMarnineViewer
        intersection_poly = geom.Intersection(poly_filter) 
        
        # Get the ring of that intersection polygon
        p_ring = intersection_poly.GetGeometryRef(0) 
        
        # Determine how many vertices there are in the polygon
        points = p_ring.GetPointCount()
        vertex = 'pts={' # String to hold the vertices
        # Cycle through the vertices and store them as a string
        for p1 in xrange(points):
            lon, lat, z = p_ring.GetPoint(p1)
            p_x,p_y = p(lon, lat)
            vertex += str(p_x-x_origin) + ','+ str(p_y-y_origin)
            if (p1!=points-1):
                vertex += ':'
        t_lvl = feature.GetField(0) # Get threat level
        
        # Change the Color of the point based on the Threat Level
        if t_lvl == 5:
            color = 'edge_color=black, vertex_color=black'
        if t_lvl == 4:
            color = 'edge_color=red,vertex_color=red'
        elif t_lvl == 3:
            color = 'edge_color=orange,vertex_color=orange'
        elif t_lvl == 2:
            color = 'edge_color=yellow,vertex_color=yellow'
        elif t_lvl == 1:
            color = 'edge_color=blue,vertex_color=brown'
        elif t_lvl == 0:
            color = 'edge_color=green,vertex_color=green'
        if points != 0:
            comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color, pymoos.time()+t)
#            print pymoos.time()+t
        feature = layer.GetNextFeature()
        
##---------------------------------------------------------------------------##
    ## Print out the lines
    ds = driver.Open(file_line, 0)
    
    # There is only one layer in each file and we want to open it
    layer = ds.GetLayer()
    
    # Filter the line layer to only include features within the area in 
    #   pMarnineViewer
    layer.SetSpatialFilter(poly_filter)
    feature = layer.GetNextFeature()
    while feature:
        line = feature.GetGeometryRef() # line from shapefile
        # Get the interesection of the line from the shapefile and the
        #   outline of tiff from pMarnineViewer
#        intersection_line = line.Intersection(poly_filter)
        intersection_line = poly_line_intersect(poly_filter, line)

        points = intersection_line.GetPointCount()

        vertex = 'pts={' # String to hold the vertices
        # Cycle through the vertices and store them as a string
        for p2 in xrange(points):
            lon, lat, z = intersection_line.GetPoint(p2)
            p_x,p_y = p(lon, lat)
            vertex += str(p_x-x_origin) + ','+ str(p_y-y_origin)
            if (p2!=points-1):
                vertex += ':'
        t_lvl = feature.GetField(0) # Get threat level
        
        # Change the Color of the point based on the Threat Level
        if t_lvl == 5:
            color = 'edge_color=black, vertex_color=black'
        if t_lvl == 4:
            color = 'edge_color=red,vertex_color=red'
        elif t_lvl == 3:
            color = 'edge_color=orange,vertex_color=orange'
        elif t_lvl == 2:
            color = 'edge_color=yellow,vertex_color=yellow'
        elif t_lvl == 1:
            color = 'edge_color=blue,vertex_color=brown'
        elif t_lvl == 0:
            color = 'edge_color=green,vertex_color=green'
        if points != 0:
            comms.notify('VIEW_SEGLIST', vertex+'},vertex_size=2.5,edge_size=2,'+color, pymoos.time()+t)
#            print pymoos.time()-t
        feature = layer.GetNextFeature()
        
if __name__ == "__main__":
    main()

        # Change the Type of the marker based on the type of obstacle
#        if obs_type == 'Rock':
#            color = 'vertex_color=red,'
#            label = label+obs_type
#            
#        elif obs_type == 'Soundg':
#            label = label+'S'
#            color = 'vertex_color=yellow,'
#            
#        elif obs_type == 'OBST':
#            label = label+obs_type
#            color = 'vertex_color=orange,'
#            
#        elif obs_type == 'Wreck':
#            label = label+obs_type
#            color = 'vertex_color=pink,'
#            
#        elif obs_type == 'Land':
#            label = label+obs_type
#            color = 'vertex_color=green,'
#        
#        elif obs_type in ['BOYISD', 'BOYSAW','BOYSPP', 'BOYLAT']:
#            label = label+obs_type
#            color = 'vertex_color=purple,'        
#        
#        elif obs_type in ['BCNLAT', 'BCNSPP']:
#            label = label+obs_type
#            color = 'vertex_color=black,'
#            
#        elif obs_type == 'Dock':
#            label = label+obs_type
#            color = 'vertex_color=brown,'