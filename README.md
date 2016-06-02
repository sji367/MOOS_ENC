# MOOS_ENC
This repository is a storage of MOOS applications and behaviors that use ENCs to give a ASV nautical chart awareness.

alpha.moos - Simple mission file in Portsmouth, NH

alpha.bhv  - Behavior file for that simple mission

ENC_converter.py - Takes a raw ENC file and converts it to a shapefiles by geometry. Stores the lat, long, obstacle type and threat level for each obstacle. Uses GDAL/ORG.

ENC_Print.py     - Python MOOS Application that takes the shape file created by the ENC converter application and prints out each obstacle in the ENC to pMarineViewer. Uses GDAL/ORG and pymoos.

ENC_Search1.py   - Python MOOS Application that takes the shape file created by the ENC converter application and searches around the ASV to determine if there are obstacles in the immediate vicinity of the ASV. If it finds any obstacles, it posts information on each obstacle to be used in obstacle avoidance. Uses GDAL/ORG and pymoos.


BHV_OA.cpp - MOOS Behavior file for obstacle avoidance using ENCs. Takes the information from search application to create IvP functions to avoid obstacles. At this point it uses the ZAIC tool and is not working correctly when there are multiple obstacles in the search area. 

BHV_OA.h   - Header file for the obstacle avoidance behavior
