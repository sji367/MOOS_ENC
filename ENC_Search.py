# -*- coding: utf-8 -*-
"""
Created on Mon Apr 18 12:51:02 2016

@author: mapper
"""

# Python-MOOS Bridge
import pymoos

# Used for delays
import time

# pyproj is a library that converts lat long coordinates into UTM
import pyproj 

# GDAL is a library that reads and writes shapefiles
from osgeo import ogr

# Numpy is a useful tool for 
import numpy as np

comms = pymoos.comms()

# Calculate the origin 
p = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')

LatOrigin  = 43.071959194444446
LongOrigin = -70.711610833333339 
x_origin,y_origin = p(LongOrigin, LatOrigin)

# Register for updates of the MOOS variables NAV_X and NAV_Y once every second
def on_connect():
    comms.register('NAV_X',1)
    comms.register('NAV_Y',1)
    comms.register('NAV_HEADING',1)
    return True
#==============================================================================
# Function to determine what the important information should be outputed for
#   each polygon in the search area. This function only works if there are 
#   vertices of the polygon in the search area.
#==============================================================================
def polygon(ASV_X,ASV_Y, heading, feature, intersect, TF_intersect, cntr):
    if (TF_intersect):
        geom = intersect
    else:
        geom = feature.GetGeometryRef()
        
    x1 = 0
    y1 = 0
    x2 = 0
    y2 = 0
    x_small = 9999
    y_small = 9999
    
    min_angle = 360
    max_angle = 0
    min_dist = 9999 
    min_dist_angle = 999
    
    d_min = 0
    d_max = 0
    min_dist = 9999
    
    ring = geom.GetGeometryRef(0)
    num_points =  ring.GetPointCount()
    # Cycle through to find the matching x and y coordinates
    for i in range (0, num_points):
        lon = ring.GetX(i)#np.around(ring.GetX(i), decimals=6)
        lat = ring.GetY(i)#np.around(ring.GetY(i), decimals=6)
        ptx, pty = p(lon, lat)
        
        ptx += -x_origin
        pty += -y_origin
        
        d = np.sqrt(np.square(ASV_X-ptx)+np.square(ASV_Y-pty))
        angle = np.arctan2(pty-ASV_Y,ptx-ASV_X)*180/np.pi      
        
        # Make sure that the angle are positive
        if angle <0:
            angle += 360
        angle += heading
        
        # Make sure that the angle are below 360
        if angle >360:
            angle += -360
            
        # Determine if the angle is the minimum angle
        if angle < min_angle:
            min_angle = angle
            d_min = d
            x1 = ptx
            y1 = pty
        
        # Determine if the angle is the maximum angle
        if angle > max_angle:
            max_angle = angle
            d_max = d
            x2 = ptx
            y2 = pty
        
        # Check if this point is the points match the envelope and to find  
        #   the x and y coordinates is the shortest distance
        if min_dist > d:
            min_dist = d
#            print "distance: %f" %d
            min_dist_angle = angle
            x_small = ptx
            y_small = pty
        
    # Post these points to pMarineViewer
    pt1 = 'x='+str(x1)+',y='+str(y1)+',vertex_size=8,vertex_color=white,label=pt1_'+str(cntr)
    comms.notify('VIEW_POINT', pt1, pymoos.time())
    time.sleep(.001)
    pt2 = 'x='+str(x2)+',y='+str(y2)+',vertex_size=8,vertex_color=white,label=pt2_'+str(cntr)
    comms.notify('VIEW_POINT', pt2, pymoos.time())
    
    # Find other useful info on the obstacle
    t_lvl = feature.GetField(0)
    obs_type = feature.GetField(1)
    
    if num_points:
    # t_lvl,type @ min_ang,max_ang,min_dist_ang @ min_ang_dist,max_ang_dist,min_dist
    #   To convert the calculated angle to the one that relates to the one that
    #   MOOS outputs, you have to use the formula: MOOS_ang = -(calc_ang-90)
        poly = str(t_lvl)+','+str(obs_type)+'@'+str(np.mod(-(max_angle-heading)+90, 360))+','+str(np.mod(-(min_angle-heading)+90, 360))+','+str(np.mod(-(min_dist_angle-heading)+90, 360))+'@'+str(d_min)+','+str(d_max)+','+str(min_dist)
    else:
        poly = "No points"    
    
    return poly
            
#==============================================================================
# This program uses the X and Y cooridinates from the ASV and filters out all 
#   of the points from the ENC database that are in a predetermined search
#   radius. It then outputs information need for obstacle avoidance to the
#   MOOSDB as a string.
#==============================================================================
def main():
    

    #easily mark all of the output and input files
    file_pnt = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_pnt2.shp'  
    file_poly = '/home/mapper/Desktop/MOOS_ENC/Data/US5NH02M/Shape/ENC_poly2.shp'  
    
    # Get the driver and open the point file first
    driver = ogr.GetDriverByName('ESRI Shapefile')
    ds = driver.Open(file_pnt, 0)
    ds_poly = driver.Open(file_poly, 0) # open Polygon File
    
    # There is only one layer in each file and we are just opening it to see how
    #   features there are in the layer
    layer = ds.GetLayer()
    layer_poly = ds_poly.GetLayer() # Open the only layer in the Polygon File
    
    # Register for NAV_X and NAV_Y
    comms.set_on_connect_callback(on_connect);
    comms.run('localhost',9000,'Search')
    NAV_X, NAV_Y, NAV_HEAD = [],[],[]
    X = Y = 0
    max_cntr = 1
    max_cntr_poly = 1
    while True:
        time.sleep(.001)
        ## Update the values of the ASV position (x,y) - they are in meters
        info = comms.fetch()
        # Store all values of the ASV's position
        for x in info:
            if x.is_double():
                print x.name()+ ': ' +str(x.double())
                if x.name()=='NAV_X':
                    NAV_X.append(x.double())
                elif x.name()=='NAV_Y':
                    NAV_Y.append(x.double())
                elif x.name()=='NAV_HEADING':
                    NAV_HEAD.append(x.double())
        ## If there is a new position of the ASV, then filter the data and 
        #   highlight the ones in the search area
        # Check to see if either of the list are empty
        if len(NAV_X) != 0 and len(NAV_Y)!= 0 and len(NAV_HEAD)!=0:
            # Clear previous values from ring and poly and remove the old
            #   spatial filter
            layer.SetSpatialFilter(None)
            layer_poly.SetSpatialFilter(None)
            ring_filter = None
            poly_filter = None
                
            # Create the baseline for the search area polygon 
            ring_filter = ogr.Geometry(ogr.wkbLinearRing)
            poly_filter = ogr.Geometry(ogr.wkbPolygon)            
            x_norm = NAV_X.pop()
            y_norm = NAV_Y.pop()
            heading = NAV_HEAD.pop()
            X = x_norm+x_origin
            Y = y_norm+y_origin
            
            # Search area should be a function of the vessel size and current 
            #   speed
            search_dis = 50            
            
            # Convert the positions to latitude and longitude
            E_long_search,N_lat_search= p(X+search_dis, Y+search_dis, inverse=True)
            W_long_search,S_lat_search= p(X-search_dis, Y-search_dis, inverse=True)
#            print 'N_lat: %f, E_long: %f' %(N_lat_search, E_long_search)
#            print 'S_lat: %f, W_long: %f' %(S_lat_search, W_long_search)
            
            # Build a ring of the points for the search area polygon
            ring_filter.AddPoint(W_long_search, N_lat_search)
            ring_filter.AddPoint(E_long_search, N_lat_search)
            ring_filter.AddPoint(E_long_search, S_lat_search)
            ring_filter.AddPoint(W_long_search, S_lat_search)
            ring_filter.AddPoint(W_long_search, N_lat_search)
            poly_filter.AddGeometry(ring_filter) # Add the ring to the previously created polygon         
            
            # Show the Search Radius Polygon on pMarnineViewer
            s_poly_vert = 'pts={'+str(x_norm-search_dis)+','+ str(y_norm+search_dis)+':'+ str(x_norm+search_dis)+','+str(y_norm+search_dis)+':'+ str(x_norm+search_dis)+','+str(y_norm-search_dis)+':' +str(x_norm-search_dis)+','+str(y_norm-search_dis)+'},'
            comms.notify('VIEW_POLYGON', s_poly_vert+'label=Search,edge_size=10,vertex_size=1,edge_color=red', pymoos.time())
            # Filter out data to determine the search area and print out the 
            #   number of potential hazards in the imediate vacinity 
            feature_poly = layer_poly.GetFeature(1)
            layer.SetSpatialFilter(poly_filter)
            layer.SetAttributeFilter("t_lvl!=0")
            layer_poly.SetSpatialFilter(poly_filter)
            layer_poly.SetAttributeFilter("t_lvl!=0")
            feature = layer.GetNextFeature()
#            feature_poly = layer_poly.GetFeature(0)
            print 'Number of Hazards (point): %d' %layer.GetFeatureCount()
            print 'Number of Hazards (polygon): %d \n' %layer_poly.GetFeatureCount()
            
            # This counter is used to count the number of features in the 
            #   search radius. This is then used to determine if any of the old
            #   hazard polygons need to be removed.
            counter = 1
            counter_poly = 1
            # Counter to determine how many obstacles that are above threat 
            #   level 0 in the search radius
            num_obs = 0
            
            # Intialize the obstacle string
            obs_pos = ''
            
###############################################################################
##########             Cycle through the point obstacles             ##########
###############################################################################
            # Highlight all point features in search radius in pMarineViewer 
            #   and store them into a MOOS variable called "Obstacles"
            feature = layer.GetNextFeature() 
            while feature:
                time.sleep(.001)
                geom_point = feature.GetGeometryRef()
                x,y = p(geom_point.GetX(), geom_point.GetY())
                new_x = x-x_origin
                new_y = y-y_origin
                pos = 'x='+str(new_x)+',y='+str(new_y)
                poly_search = 'format=radial,'+pos+',radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label='+str(counter)
                comms.notify('VIEW_POLYGON', poly_search, pymoos.time())
                                
                # Store information for the obstacle to be used later to post to the MOOSDB
                # x,y,t_lvl,type 
                if feature.GetField(0) !=0:
                    # if it isnt the first obstacle put a ! at the end
                    if num_obs!=0:
                        obs_pos += '!'
                    num_obs += 1
                    obs_pos += pos+','+ str(feature.GetField(0))+','+ str(feature.GetField(1))

                # Go to the next feature and increment the counter
                feature = layer.GetNextFeature() 
                counter += 1
            #print 'Number of Hazards (point): %d, %d' %(layer.GetFeatureCount(), num_obs)
            # Output to the MOOSDB a list of obstacles
            #   ASV_X,ASV_Y : # of Obstacles : x,y,t_lvl,type : x,y,t_lvl,type : ...
#            if num_obs != 0:
            obstacles = str(x_norm)+','+str(y_norm)+','+str(heading)+':'+str(num_obs)+':'+obs_pos
            comms.notify('Obstacles', obstacles, pymoos.time())            
            # Determine if a new polygon was used
            if max_cntr < counter:
                max_cntr = counter  
                
            # Remove highlighted point obstacles (shown as polygons) from 
            #   pMarineViewer
            for i in range(counter, max_cntr):
                time.sleep(.002)
                poly = 'format=radial,x= 0,y=0,radius=25,pts=8,edge_size=5,vertex_size=2,active=false,label='+str(i)
                comms.notify('VIEW_POLYGON', poly, pymoos.time())
                
###############################################################################
##########                 Cycle through the polygons                ##########
###############################################################################
            # Initialize the polygon strings
            poly_str = ''
            poly_info = ''
            feature_poly = layer_poly.GetNextFeature()
            while feature_poly:
                geom_poly = feature_poly.GetGeometryRef() # Polygon from shapefile
                # Get the interesection of the polygon from the shapefile and
                #   the outline of tiff from pMarnineViewer
                intersection_poly = geom_poly.Intersection(poly_filter) 
                
                # Get the ring of that intersection polygon
                p_ring = intersection_poly.GetGeometryRef(0) 
                
                # There are two potential cases - There are vertices of the 
                #   polygon within the search area and There are no vertices of
                #   the polygon within the search area.
                
                # Case 1: There are vertices within the search area therefore 
                #   we will give the polygon string function the intersection
                #   of the polygon and the search area
                if p_ring:
                    poly_str = polygon(x_norm, y_norm, heading, feature_poly, intersection_poly, True,counter_poly)
                    
                # Case 2: there are no points in the the search window, 
                #   therefore we are temperarily giving the entire polygon to 
                #   the polygon function.
                else:
                    poly_str = polygon(x_norm, y_norm, heading, feature_poly, False, False, counter_poly)
   
                # If it is not the first polygon, then add a '!' to the end of 
                #   the  string.
                if poly_str!="No points":
                    if counter_poly==1:
                        poly_info = poly_str
                    elif counter_poly>1:
                        poly_info += '!'
                        poly_info += poly_str
                    # Increment counter
                    counter_poly += 1
                    
                # Go to the next polygon
                feature_poly = layer_poly.GetNextFeature()
                

            # Post an update if there are polygon obstacles
            if (layer_poly.GetFeatureCount()>1):
                poly_obs = str(x_norm)+','+str(y_norm)+','+str(heading)+':'+str(layer_poly.GetFeatureCount()-1)+':'+poly_info
                comms.notify('Poly_Obs', poly_obs, pymoos.time())  
            else:
                poly_obs = str(x_norm)+','+str(y_norm)+','+str(heading)+':'+str(layer_poly.GetFeatureCount()-1)
                comms.notify('Poly_Obs', poly_obs, pymoos.time())
            # Determine if a new polygon was used
            if max_cntr_poly < counter_poly:
                max_cntr_poly = counter_poly    
                
            # Remove highlighted line/polygon obstacles (shown as seglist) from 
            #   pMarineViewer
            for i in range(1, counter_poly):
                time.sleep(.002)
                poly = 'x=1,y1,active=false,label=pt1_'+str(i)
                poly = 'x=1,y1,active=false,label=pt2_'+str(i)
                comms.notify('VIEW_POINT', poly, pymoos.time())
                
            
   
        # MOOS freaks out when nothing is posted to the DB so post this dummy
        #   variable to avoid this problem if nothing was posted during the l
        #   last cycle
        else:
            comms.notify('dummy_var','',pymoos.time())

if __name__ == "__main__":
    main()        
