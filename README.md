# MOOS_ENC
This repository is a storage of MOOS applications and behaviors that use ENCs to give a ASV nautical chart awareness.

alpha.moos - Simple mission file in Portsmouth, NH

alpha.bhv  - Behavior file for that simple mission

ENC_converter.py - Takes a raw ENC file and converts it to a shapefiles by geometry. Stores the lat, long, obstacle type and threat level for each obstacle. Uses GDAL/ORG.

ENC_Print.py     - Python MOOS Application that takes the shape file created by the ENC converter application and prints out each obstacle in the ENC to pMarineViewer. Uses GDAL/ORG and pymoos.

ENC_Search.py   - Python MOOS Application that takes the shape file created by the ENC converter application and searches around the ASV to determine if there are obstacles in the immediate vicinity of the ASV. If it finds any obstacles, it posts information on each obstacle to be used in obstacle avoidance. It works for both polygon and point obstacles. Uses GDAL/ORG and pymoos.

BHV_OA.cpp - MOOS Behavior file for obstacle avoidance using ENCs (only obstacles with point geometry). Takes the information from search application to create IvP functions to avoid obstacles with point geometry. It uses the ZAIC Vector tool. 

BHV_OA.h   - Header file for the obstacle avoidance behavior for obstacles with point geometry.

BHV_OA_poly.cpp - MOOS Behavior file for obstacle avoidance using ENCs (only obstacles with polygon geometry). Takes the information from search application to create IvP functions to avoid obstacles with polygon geometry. It uses the ZAIC Vector tool. 

BHV_OA_poly.h   - Header file for the obstacle avoidance behavior for obstacles with polygon geometry.
