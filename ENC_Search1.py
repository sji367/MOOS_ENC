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

#==============================================================================
# This program uses the X and Y cooridinates from the ASV and filters out all 
#   of the points from the ENC database that are in a predetermined search
#   radius. It then outputs the 
#==============================================================================

comms = pymoos.comms()

# Register for updates of the MOOS variables NAV_X and NAV_Y once every second
def on_connect():
    comms.register('NAV_X',1)
    comms.register('NAV_Y',1)
    comms.register('NAV_HEADING',1)
    return True

def main():
    t = 0#pymoos.time()
    
    # Calculate the origin 
    p = pyproj.Proj(proj='utm', zone=19, ellps='WGS84')
    
    LatOrigin  = 43.071959194444446
    LongOrigin = -70.711610833333339 
    x_origin,y_origin = p(LongOrigin, LatOrigin)

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
            ring_filter = poly_filter = None
                
            # Create the baseline for the search area polygon 
            ring_filter = ogr.Geometry(ogr.wkbLinearRing)
            poly_filter = ogr.Geometry(ogr.wkbPolygon)            
            x_norm = NAV_X.pop()
            y_norm = NAV_Y.pop()
            heading = NAV_HEAD.pop()
            X = x_norm+x_origin
            Y = y_norm+y_origin
            
            # Convert the positions to latitude and longitude
            E_long_search,N_lat_search= p(X+50, Y+50, inverse=True)
            W_long_search,S_lat_search= p(X-50, Y-50, inverse=True)
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
            s_poly_vert = 'pts={'+str(x_norm-50)+','+ str(y_norm+50)+':'+ str(x_norm+50)+','+str(y_norm+50)+':'+ str(x_norm+50)+','+str(y_norm-50)+':' +str(x_norm-50)+','+str(y_norm-50)+'},'
            comms.notify('VIEW_POLYGON', s_poly_vert+'label=Search,edge_size=10,vertex_size=1,edge_color=red', pymoos.time()+t)
            # Filter out data to determine the search area and print out the 
            #   number of potential hazards in the imediate vacinity 
            layer.SetAttributeFilter("t_lvl!=0")
            layer.SetSpatialFilter(poly_filter)
            layer_poly.SetAttributeFilter("t_lvl!=0")
            layer_poly.SetSpatialFilter(poly_filter)
            feature = layer.GetNextFeature()
            feature_poly = layer_poly.GetNextFeature()
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
            # Highlight all point features in search radius in pMarineViewer 
            #   and store them into a MOOS variable called "Obstacles"
            for i in range(layer.GetFeatureCount()):
                time.sleep(.001)
                geom_point = feature.GetGeometryRef()
                x,y = p(geom_point.GetX(), geom_point.GetY())
                new_x = x-x_origin
                new_y = y-y_origin
                pos = 'x='+str(new_x)+',y='+str(new_y)
                poly_search = 'format=radial,'+pos+',radius=25,pts=8,edge_size=5,vertex_size=2,edge_color=aquamarine,vertex_color=aquamarine,label='+str(counter)
                comms.notify('VIEW_POLYGON', poly_search, pymoos.time()+t)
                                
                # Store information for the obstacle to be used later to post to the MOOSDB
                # x,y,t_lvl,type 
                if feature.GetField(0) !=0:
                    # if it isnt the last obstacle put a ! at the end
                    if num_obs!=0:
                        obs_pos += '!'
                    num_obs += 1
                    obs_pos += pos+','+ str(feature.GetField(0))+','+ str(feature.GetField(1))

                # Go to the next feature and increment the counter
                feature = layer.GetNextFeature() 
                counter += 1
            # Output to the MOOSDB a list of obstacles
            #   ASV_X,ASV_Y : # of Obstacles : x,y,t_lvl,type : x,y,t_lvl,type : ...
#            if num_obs != 0:
            obstacles = str(x_norm)+','+str(y_norm)+','+str(heading)+':'+str(num_obs)+':'+obs_pos        
            comms.notify('Obstacles', obstacles, pymoos.time()+t)            
            # Determine if a new polygon was used
            if max_cntr < counter:
                max_cntr = counter

            ## Cycle throught the polygons 
            while feature_poly:
                geom_poly = feature_poly.GetGeometryRef() # Polygon from shapefile
                # Get the interesection of the polygon from the shapefile and
                #   the outline of tiff from pMarnineViewer
                intersection_poly = geom_poly.Intersection(poly_filter) 
            
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
                poly = vertex+'},edge_color=yellow,label='+str(counter_poly)
                if points !=0:
                    comms.notify('VIEW_SEGLIST', poly, pymoos.time()+t)
                counter_poly += 1
                feature_poly = layer_poly.GetNextFeature()
                
            # Determine if a new polygon was used
            if max_cntr_poly < counter_poly:
                max_cntr_poly = counter_poly    
            # Move polygon off screen - temperary fix for removing highlighted 
            #   obstacles (shown as polygons) from pMarineViewer
            for i in range(counter, max_cntr+1):
                time.sleep(.002)
                poly = 'format=radial,x= 5000,y=5000,radius=25,pts=8,edge_size=5,vertex_size=2,label='+str(i)
                comms.notify('VIEW_POLYGON', poly, pymoos.time()+t)
            
            # Move polygon off screen - temperary fix for removing highlighted 
            #   obstacles (shown as polygons) from pMarineViewer
            for i in range(counter_poly, max_cntr_poly+1):
                time.sleep(.002)
                poly = 'pts={5000,5000: 5001,5001},label='+str(i)
                comms.notify('VIEW_SEGLIST', poly, pymoos.time()+t)
                
            
   
        # MOOS freaks out when nothing is posted to the DB so post this dummy
        #   variable to avoid this problem if nothing was posted during the l
        #   last cycle
        else:
            comms.notify('dummy_var','',pymoos.time()+t)

if __name__ == "__main__":
    main()        