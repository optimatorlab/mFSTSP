#!/usr/bin/env python

from __future__ import division
from math import *
from collections import defaultdict
import os


# =================================================================
radius_of_earth = 6378100.0	# [meters]
DIST_TOL = 1.0	# [meters]	If we're within DIST_TOL meters of the target, just assume we're actually at the target.
ALT_TOL = 1.0 # [meters]

MODE_CAR 		= 1
MODE_BIKE 		= 2
MODE_WALK 		= 3
MODE_FLY 		= 4


METERS_PER_MILE = 1609.34
# =================================================================


# This file contains a function for calculating UAV travel time.


def calcMultirotorTravelTime(takeoffSpeed, cruiseSpeed, landSpeed, yawRateDeg, initAlt, flightAlt, goalAlt, initLatDeg, initLongDeg, goalLatDeg, goalLongDeg, initHeadingDeg, goalHeadingDeg):

	# Example:
	# calcMultirotorTravelTime(20.0, 30.0, 10.0, 15.0, 0.0, 50.0, 0.0, 45.0, 45.0, 55.0, 55.0, -45.0, -361)
	
	# NOTES:
	#	* lat/long values are in units of [DEGREES]
	#	* altitude values are in units of [meters]
	#	* turn radius is in units of [meters]
	#	* yaw rate is in units of [DEGREES/second]
	#	* heading is in units of [DEGREES]
	#		- If initHeadingDeg == -361, we'll assume we need to rotate 180-degrees
	#		- If goalHeadingDeg == -361, we'll ignore any rotation required at the destination
	
	# Convert units to radians:
	yawRateRad = yawRateDeg*(pi/180)
	initLatRad = initLatDeg*(pi/180)
	initLongRad = initLongDeg*(pi/180)
	goalLatRad = goalLatDeg*(pi/180)
	goalLongRad = goalLongDeg*(pi/180)
	initHeadingRad = initHeadingDeg*(pi/180)
	goalHeadingRad = goalHeadingDeg*(pi/180)
	
	takeoffTime = 0.0
	flyTime = 0.0 
	landTime = 0.0 
	totalTime = 0.0
	
	takeoffDistance = 0.0
	flyDistance = 0.0
	landDistance = 0.0
	totalDistance = 0.0

	# See if we're actually "close enough" to the destination
	distanceToGoal = groundDistanceStraight(initLatRad, initLongRad, goalLatRad, goalLongRad)
	if (distanceToGoal <= DIST_TOL):
		# Go ahead and treat the initial and goal lat/long values to be identical
		initLatRad = goalLatRad
		initLongRad = goalLongRad

	if (abs(initAlt - goalAlt) <= ALT_TOL):
		# Go ahead and treal the initial and goal altitudes to be identical
		initAlt = goalAlt 

	# NOTE: It's possible that our initial and goal locations are the same. 
	# In this case, we don't need any travel (esp. takeoff/landing time)
	if (([initLatRad, initLongRad, initAlt] == [goalLatRad, goalLongRad, goalAlt]) and (goalHeadingDeg == -361)):
		totalTime = 0.0
	else:
		# 1) Adjust altitude (e.g., Take off)
		#	 NOTE: Only do this if our init/goal coordinates differ (otherwise we're not prepping to fly anywhere)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			deltaAlt = flightAlt - initAlt
			totalTime += (abs(deltaAlt)/takeoffSpeed)
			takeoffTime += (abs(deltaAlt)/takeoffSpeed)
			takeoffDistance += abs(deltaAlt)
			totalDistance += abs(deltaAlt)
	
		# 2) Rotate towards target (yaw)
		#	 NOTE: Only do this if our init/goal coordinates differ (otherwise we've already reached the destination and we don't need to rotate here)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			if (initHeadingDeg <= -361):
				# We don't know the initial heading.
				# We're assuming we'll have to turn 180-degrees (worst case)
				totalTime += pi/yawRateRad
				takeoffTime += pi/yawRateRad
			else:
				# What angle is required to travel directly from the current location to the goal location?
				#	 See http://www.movable-type.co.uk/scripts/latlong.html
				y = sin(goalLongRad - initLongRad) * cos(goalLatRad)
				x = cos(initLatRad)*sin(goalLatRad) - sin(initLatRad)*cos(goalLatRad)*cos(goalLongRad-initLongRad)
				headingStrRad = (atan2(y, x) + 2*pi) % (2*pi)	# In the range [0,2*pi]

				# What is the difference between the current heading and the straight-line bearing?
				# The following formula tells us how far we'd have to rotate (in radians) if we rotate CCW:
				deltaHeadingRad = (initHeadingRad - headingStrRad) % (2*pi)

				# Should we rotate CW or CCW?
				if (deltaHeadingRad <= pi):
					# The shortest rotation will be CCW.
					rotateCCW = 1		# Set to true
				else:
					# We'd prefer to rotate CW instead.
					rotateCCW = 0		# Set to false
					# When rotating CW, the actual required rotation angle is less than pi radians:
					deltaHeadingRad = 2*pi - deltaHeadingRad
	
				# Rotate deltaHeadingRad radians
				totalTime += deltaHeadingRad/yawRateRad
				takeoffTime += deltaHeadingRad/yawRateRad
	
		# 3) Fly to target
		myDistance = groundDistanceStraight(initLatRad, initLongRad, goalLatRad, goalLongRad)
		totalTime += myDistance/cruiseSpeed
		flyTime += myDistance/cruiseSpeed
		flyDistance += myDistance
		totalDistance += myDistance
		
		# 4) Rotate at target to desired heading (if applicable)
		if (goalHeadingDeg <= -361):
			# We don't care about the goal heading
			totalTime += 0.0
			landTime += 0.0
			
		else:
			# What angle is required to travel directly from the current location to the goal location?
			#	 See http://www.movable-type.co.uk/scripts/latlong.html
			y = sin(goalLongRad - initLongRad) * cos(goalLatRad)
			x = cos(initLatRad)*sin(goalLatRad) - sin(initLatRad)*cos(goalLatRad)*cos(goalLongRad-initLongRad)
			headingStrRad = (atan2(y, x) + 2*pi) % (2*pi)	# In the range [0,2*pi]

			# What is the difference between the desired heading and the straight-line bearing?
			# The following formula tells us how far we'd have to rotate (in radians) if we rotate CCW:
			deltaHeadingRad = (headingStrRad - goalHeadingRad) % (2*pi)

			# Should we rotate CW or CCW?
			if (deltaHeadingRad <= pi):
				# The shortest rotation will be CCW.
				rotateCCW = 1		# Set to true
			else:
				# We'd prefer to rotate CW instead.
				rotateCCW = 0		# Set to false
				# When rotating CW, the actual required rotation angle is less than pi radians:
				deltaHeadingRad = 2*pi - deltaHeadingRad
	
			# Rotate deltaHeadingRad radians
			totalTime += deltaHeadingRad/yawRateRad
			landTime += deltaHeadingRad/yawRateRad
	
		# 5) Adjust altitude (e.g., Land)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			# We haven't yet reached the destination, so we'll have to change from flightAlt to goalAlt
			deltaAlt = flightAlt - goalAlt
			totalTime += (abs(deltaAlt)/landSpeed)
			landTime += (abs(deltaAlt)/landSpeed)
			landDistance += abs(deltaAlt)
			totalDistance += abs(deltaAlt)
		else:
			# We're already at the destination, and we might have started landing
			# initAlt describes the "current" altitude
			deltaAlt = initAlt - goalAlt
			totalTime += (abs(deltaAlt)/landSpeed)
			landTime += (abs(deltaAlt)/landSpeed)
			landDistance += abs(deltaAlt)
			totalDistance += abs(deltaAlt)

	
	return [takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance]

	
	
def groundDistanceStraight(lat1, long1, lat2, long2):
	# Calculate Distance to from point 1 to point 2, in [meters]:
	# This is a straight-line distance, ignoring altitude changes and turning

	# NOTES:
	#	* lat/long values are in units of [RADIANS]

	distance = 2*radius_of_earth*asin( sqrt( pow(sin((lat2 - lat1)/2),2) + cos(lat1)*cos(lat2)*pow(sin((long2-long1)/2),2) ))

	return (distance)
