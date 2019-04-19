#!/usr/bin/env python

from __future__ import division
import urllib2
import json
import numpy
#from geopy.distance import great_circle
from math import *
from collections import defaultdict
import os


# =================================================================
# FIXME -- These should be in a config file somewhere: 
radius_of_earth = 6378100.0	# [meters]
DIST_TOL = 1.0	# [meters]	If we're within DIST_TOL meters of the target, just assume we're actually at the target.
ALT_TOL = 1.0 # [meters]

MODE_CAR 		= 1
MODE_BIKE 		= 2
MODE_WALK 		= 3
MODE_FLY 		= 4

USER_KEY = os.environ['MAPQUESTKEY']



METERS_PER_MILE = 1609.34
# =================================================================


# This file contains a number of functions for calculating time/distance to target.

# http://stackoverflow.com/questions/635483/what-is-the-best-way-to-implement-nested-dictionaries-in-python
def make_dict():
	return defaultdict(make_dict)
		
	
class make_myShapepoints:
	def __init__(self, arrivalTimeSec, latDeg, longDeg, legIndex):
		self.arrivalTimeSec	= arrivalTimeSec
		self.latDeg 		= latDeg
		self.longDeg		= longDeg
		self.legIndex		= legIndex

def getShapepoints(startLatDeg, startLongDeg, endLatDeg, endLongDeg, USER_KEY, travelMode, expDurationSec = None):
	# Returns dictionary of all shapepoints from start to end.
	
	# startCoords = '38.205024,-85.75077'	 -- Text string, NO SPACES
	# endCoords = '38.210687,-85.764597'	 -- Text string, NO SPACES
	startCoords = '%f,%f' % (startLatDeg, startLongDeg)
	endCoords = '%f,%f' % (endLatDeg, endLongDeg)

	if (travelMode == MODE_BIKE):
		transportMode = 'bicycle'
	elif (travelMode == MODE_WALK):
		transportMode = 'pedestrian'
	else:
		# We'll assume travelMode == MODE_CAR
		transportMode = 'fastest'
		       
	# assemble query URL 
	newUrl = 'http://open.mapquestapi.com/directions/v2/route?key={}&routeType={}&from={}&to={}'.format(USER_KEY, transportMode, startCoords, endCoords)
	newUrl += '&doReverseGeocode=false&fullShape=true'
	
	# print newUrl
		
	# make request and receive response
	request = urllib2.Request(newUrl)
	response = urllib2.urlopen(request)
	
	# convert JSON response to dictionary
	data = json.load(response)
	
	#print data
	#print '\n'
	
	# retrieve info for each leg: start location, length, and time duration
	lats = [data['route']['legs'][0]['maneuvers'][i]['startPoint']['lat'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	lngs = [data['route']['legs'][0]['maneuvers'][i]['startPoint']['lng'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	secs = [data['route']['legs'][0]['maneuvers'][i]['time'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	dist = [data['route']['legs'][0]['maneuvers'][i]['distance'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	
	# create list of dictionaries (one dictionary per leg) with the following keys: "waypoint", "time", "distance"
	legs = [dict(waypoint = (lats[i],lngs[i]), time = secs[i], distance = dist[i]) for i in range(0,len(lats))]
	
	# create list of waypoints (waypoints define legs)
	wayPoints = [legs[i]['waypoint'] for i in range(0,len(legs))]
	
	# get shape points (each leg has multiple shapepoints)
	shapePts = [tuple(data['route']['shape']['shapePoints'][i:i+2]) for i in range(0,len(data['route']['shape']['shapePoints']),2)]
	
	# every waypoint is included in the list of shape points. find the indices where the waypoints are.
	# this will indicate a new "leg" has begun within the list shape points.
	legIndices = [shapePts.index(wayPoints[i]) for i in range(0,len(wayPoints))]
	
	# adjust indices to make things easier
	newLegIndices = [legIndices[i]+1 if i not in (0,len(legIndices)) else legIndices[i] for i in range(0,len(legIndices))]
	
	# group shape points by leg in a list
	shapePtsByLeg = [shapePts[newLegIndices[i]:newLegIndices[i+1]] for i in range(0,len(newLegIndices)-1)]
	
	# for each leg's dictionary containing leg info, add a shapePoints key and list of points along the leg
	for i in range(0,len(shapePtsByLeg)):
		legs[i]['shapePoints'] = shapePtsByLeg[i]
		
	# legs[0]['distance'] --> scalar value describing the distance of the first leg (leg 0)
	# legs[0]['shapePoints'] --> List of shape points associated with the first leg
	#								[(38.205024, -85.75077), (38.211326, -85.751243), (38.211593, -85.751312)]
	# legs[0]['shapePoints'][1] --> Second pair of shape point coordinates for first leg.
	#								(38.205024, -85.75077)
	# legs[0]['shapePoints'][1][0] --> first coordinate of second pair -- 38.205024
	# legs[0]['time'] --> Number of seconds to travel the first leg (leg 0)


	# OPTIONAL: If the user provided a value for "expDurationSec", we'd like our 
	# output to match the total expected duration.
	if (expDurationSec != None):
		# a) Calculate mapquest's time for each leg.
		totalTimeSec = 0.0
		totalDistMiles = 0.0
		legTimePct = []
		for i in range(0,len(legs)):
			totalTimeSec += legs[i]['time']
			totalDistMiles += legs[i]['distance']
			legTimePct.append(0.0)	# Initialize

		# print 'expDurationSec = %f \t totalTimeSec = %f \t Delta = %f' % (expDurationSec, totalTimeSec, expDurationSec - totalTimeSec)
					
		# b) Calculate the percentage of time spent on each leg
		# ONLY IF THE TOTAL TIME FOR THIS O/D PAIR IS NON-ZERO
		if (float(totalTimeSec) > 0.0):
			for i in range(0,len(legs)):
				legTimePct[i] = legs[i]['time']/float(totalTimeSec)
		
			# c) Edit the time that SHOULD be spent on each leg (to produce a route with a 
			#	 duration of expDurationSec
			for i in range(0,len(legs)):
				# Overwrite mapquest's time for each leg (in [seconds]):
				# print 'legs[%d][time] was %f, now %f' % (i, legs[i]['time'], legTimePct[i]*expDurationSec)

				legs[i]['time'] = legTimePct[i]*expDurationSec
						
			
	myShapepoints = defaultdict(make_dict)
	
	# Snap the asset to the nearest address.
	# This will be the very first shapepoint on the first leg.
	currentTimeSec 	= 0.0
	currentLatDeg 	= legs[0]['shapePoints'][0][0]
	currentLongDeg 	= legs[0]['shapePoints'][0][1]

	spIndex = 0
	myShapepoints[spIndex] = make_myShapepoints(currentTimeSec, currentLatDeg, currentLongDeg, 0)		# The last "0" --> This is leg 0.
					
	for legIndex in range(0,len(legs)-1):
		# print "Leg %d -- %f miles in %f seconds):" % (legIndex, legs[legIndex]['distance'], legs[legIndex]['time'])
	
		# Estimate, for THIS LEG, the percentage of the overall leg distance
		# that is traveled from the previous shapepoint.

		# Initialize our calculated sum of straightline distances.  
		# This might not equal legs[legIndex]['distance']
		myTotalLegDistMiles = 0.0  
		
		myShapepointDist = []
		myShapepointDist.append([])	# Skip the first SP, which is the origin
		for j in range(1,len(legs[legIndex]['shapePoints'])):
		
			nextLatDeg 	= legs[legIndex]['shapePoints'][j][0]
			nextLongDeg	= legs[legIndex]['shapePoints'][j][1]
		
			# Calculate distance [in meters] to travel from the previous shapepoint
			# (current) to this shapepoint (next).
			# Need the coordinates to be in radians, not degrees
			distFromPrev = groundDistanceStraight(currentLatDeg*(pi/180.0), currentLongDeg*(pi/180.0), nextLatDeg*(pi/180.0), nextLongDeg*(pi/180.0))
		
			myShapepointDist.append(distFromPrev/METERS_PER_MILE)
			myTotalLegDistMiles += myShapepointDist[j]

		# We now know the sum of the distances between shapepoints for this leg, 
		# as well as the individual distances between shapepoints.
		# Now, calculate the percentages for each sub-leg.
		# NOTE: We're essentially assuming that cars travel the same speed throughout
		# this leg.
		for j in range(1,len(legs[legIndex]['shapePoints'])):
			if (legs[legIndex]['time'] > 0.0):
				# Use pct of distance to estimate arrival time

				nextLatDeg 	= legs[legIndex]['shapePoints'][j][0]
				nextLongDeg	= legs[legIndex]['shapePoints'][j][1]
				nextTimeSec = currentTimeSec + (myShapepointDist[j]/float(myTotalLegDistMiles))*legs[legIndex]['time']
			
				# Update myShapepoints dictionary		
				spIndex += 1
				myShapepoints[spIndex] = make_myShapepoints(nextTimeSec, nextLatDeg, nextLongDeg, legIndex)
						
				# Update our counters
				currentTimeSec 	= nextTimeSec	
				currentLatDeg 	= nextLatDeg
				currentLongDeg 	= nextLongDeg
		

	# Do some error-checking here:
	# 1) Is the last shapepoint at the destination location?
	if ((currentLatDeg != endLatDeg) or (currentLongDeg != endLongDeg)):
		print 'ERROR in getShapepoints (mismatch detected): \n'
		print '\t currentLatDeg = %f, endLatDeg = %f \n' % (currentLatDeg, endLatDeg)
		print '\t currentLongDeg = %f, endLongDeg = %f \n' % (currentLongDeg, endLongDeg)
	# 2) Does the total time match the (given) expected time?
	if (abs(currentTimeSec - expDurationSec) > 0.01):
		print 'ERROR in getShapepoints (mismatch detected): \n'
		print '\t currentTimeSec = %f, expDurationSec = %f \n' % (currentTimeSec, expDurationSec)
	
	# The last shapepoint might not be the exact location of node j.
	# Instantaneously move the truck to node j.
	# self.asgnDetails[assetID][tmpIndex] = make_assignment(self.assignment[assetID][asgnIndex].activity, startTime, self.assignment[assetID][asgnIndex].endNode, currentLatDeg, currentLongDeg, self.assignment[assetID][asgnIndex].endAlt, self.assignment[assetID][asgnIndex].endTime, self.assignment[assetID][asgnIndex].endNode, self.assignment[assetID][asgnIndex].endLatDeg, self.assignment[assetID][asgnIndex].endLongDeg, self.assignment[assetID][asgnIndex].endAlt, self.assignment[assetID][asgnIndex].isWithUAV)
	# tmpIndex += 1
									
	# Double-check that our calculated endTime == leg's startTime + tau[i][j]
	# print "expectedEndTime = %f, calculated endTime = %f, (exp - calc) = %f" % (expectedEndTime, endTime, expectedEndTime - endTime)
		
	return [myShapepoints, totalTimeSec, totalDistMiles]
		
		

def getMapquestLegs(startCoords, endCoords, travelMode, USER_KEY):
	# start = '38.205024,-85.75077'	 -- Text string, NO SPACES
	# end = '38.210687,-85.764597'	 -- Text string, NO SPACES
	# userKey = 'insert key here'	 -- Text string, NO SPACES
	# mode = 'fastest'
	# legs = getMapquestLegs(start, end, userKey, mode)

	if (travelMode == MODE_BIKE):
		transportMode = 'bicycle'
	elif (travelMode == MODE_WALK):
		transportMode = 'pedestrian'
	else:
		# We'll assume travelMode == MODE_CAR
		transportMode = 'fastest'
		       
	# assemble query URL 
	newUrl = 'http://open.mapquestapi.com/directions/v2/route?key={}&routeType={}&from={}&to={}'.format(USER_KEY, transportMode, startCoords, endCoords)
	newUrl += '&doReverseGeocode=false&fullShape=true'

	#newUrl += '&mapWidth=640&mapHeight=640' # provide a mapState in order to get shapePoints
	
	# print newUrl
		
	# make request and receive response
	request = urllib2.Request(newUrl)
	response = urllib2.urlopen(request)
	
	# convert JSON response to dictionary
	data = json.load(response)
	
	# print data
	
	# retrieve info for each leg: start location, length, and time duration
	lats = [data['route']['legs'][0]['maneuvers'][i]['startPoint']['lat'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	lngs = [data['route']['legs'][0]['maneuvers'][i]['startPoint']['lng'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	secs = [data['route']['legs'][0]['maneuvers'][i]['time'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	dist = [data['route']['legs'][0]['maneuvers'][i]['distance'] for i in range(0,len(data['route']['legs'][0]['maneuvers']))]
	
	# create list of dictionaries (one dictionary per leg) with the following keys: "waypoint", "time", "distance"
	legs = [dict(waypoint = (lats[i],lngs[i]), time = secs[i], distance = dist[i]) for i in range(0,len(lats))]
	
	# create list of waypoints
	wayPoints = [legs[i]['waypoint'] for i in range(0,len(legs))]
	
	# get shape points
	shapePts = [tuple(data['route']['shape']['shapePoints'][i:i+2]) for i in range(0,len(data['route']['shape']['shapePoints']),2)]
	
	# every waypoint is included in the list of shape points. find the indices where the waypoints are.
	# this will indicate a new "leg" has begun within the list shape points.
	legIndices = [shapePts.index(wayPoints[i]) for i in range(0,len(wayPoints))]
	
	# adjust indices to make things easier
	newLegIndices = [legIndices[i]+1 if i not in (0,len(legIndices)) else legIndices[i] for i in range(0,len(legIndices))]
	
	# group shape points by leg in a list
	shapePtsByLeg = [shapePts[newLegIndices[i]:newLegIndices[i+1]] for i in range(0,len(newLegIndices)-1)]
	
	# for each leg's dictionary containing leg info, add a shapePoints key and list of points along the leg
	for i in range(0,len(shapePtsByLeg)):
		legs[i]['shapePoints'] = shapePtsByLeg[i]
		
	# legs[0]['distance'] --> scalar value describing the distance of the first leg (leg 0)
	# legs[0]['shapePoints'] --> List of shape points associated with the first leg
	#								[(38.205024, -85.75077), (38.211326, -85.751243), (38.211593, -85.751312)]
	# legs[0]['shapePoints'][1] --> Second pair of shape point coordinates for first leg.
	#								(38.205024, -85.75077)
	# legs[0]['shapePoints'][1][0] --> first coordinate of second pair -- 38.205024
	# legs[0]['time'] --> Number of seconds to travel the first leg (leg 0)

	timeToDest = 0.0
	for i in range(0,len(legs)):
		timeToDest += legs[i]['time']
		
	#print legs
		
	# TESTING
	#print "TESTING:"
	#print "start: %s" % startCoords
	#print "end: %s" % endCoords	
	#for i in range(0,len(legs)-1):
	#	print "Leg %d:" % i
	#	for j in range(0,len(legs[i]['shapePoints'])):
	#		print "\t(%f,%f) - %f seconds" % (legs[i]['shapePoints'][j][0], legs[i]['shapePoints'][j][1], legs[i]['time'])
	
		
	return [legs, timeToDest]
	
def moveGroundAsset(initLatDeg, initLongDeg, goalLatDeg, goalLongDeg, legsDict, currentLegIndex, nextShapePtIndex, availTime):
		
	if (availTime == 0.0):
		# We're not given any time to actually do anything.
		# We'll pass back some dummy info (instead of wasting time calculating these distance/time values.
		distanceToGoal = 1000
		timeToGoal = 1000
		return [initLatDeg, initLongDeg, distanceToGoal, timeToGoal, availTime, currentLegIndex, nextShapePtIndex]

	else:	
		# Convert units to radians:
		# Where are we now?
		initLatRad = initLatDeg*(pi/180)
		initLongRad = initLongDeg*(pi/180)
	
		# Where do we ultimately want to be? (an actual node)	
		goalLatRad = goalLatDeg*(pi/180)
		goalLongRad = goalLongDeg*(pi/180)
	
		# Where is the next shape point?
		print "currentLegIndex = %d" % currentLegIndex
		print "nextShapePtIndex = %d" % nextShapePtIndex
		print "next (lat,long) = (%f,%f)" % (legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][0], legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][1])
		print "availTime = %f" % availTime
	
		nextLatRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][0]*(pi/180)
		nextLongRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][1]*(pi/180)
		# NOTE: The "next" represents the next shape point (not the actual/final destination)
	
		timeRemain = availTime
		
		# Initialize the new coordinates:	
		newLatRad = initLatRad
		newLongRad = initLongRad
	
		pleaseContinue = 1	# Initialize this flag

		while (pleaseContinue == 1):

			# Calculate the effective speed (in [meters/sec]) for the current leg:
			if (legsDict[currentLegIndex]['time'] > 0):
				effSpeed = (legsDict[currentLegIndex]['distance']*METERS_PER_MILE)/float(legsDict[currentLegIndex]['time'])

				# Calculate distance to travel to the next shape point:
				distToNext = groundDistanceStraight(newLatRad, newLongRad, nextLatRad, nextLongRad)
				
				# Calculate how far we can travel, given the time remaining:
				distPossible = effSpeed*timeRemain
				
				# How far can we actually travel in the time remaining?
				distanceWeCanTravel = min(distToNext, distPossible)
			else:
				# We have no distance to travel
				distToNext = 0.0
				distanceWeCanTravel = 0.0
				# distPossible is irrelevant
		
			print "I'm currently here: (%f,%f)" % (newLatRad*(180/pi), newLongRad*(180/pi))
			print "I'm headed here: (%f,%f)" % (nextLatRad*(180/pi), nextLongRad*(180/pi))
			print "distanceWeCanTravel: %f" % distanceWeCanTravel
			print "leg distance: %f [miles]" % legsDict[currentLegIndex]['distance']
			print "leg time: %f" % legsDict[currentLegIndex]['time']
			print "timeRemain: %f" % timeRemain
			print "effSpeed: %f" % effSpeed
			print "groundDistance: %f" % distToNext
		
		
			if (distToNext <= DIST_TOL):
				# We appear to have reached our next shape point.
				# Is there a next shape point for this leg?
				# [(38.205024, -85.75077), (38.211326, -85.751243), (38.211593, -85.751312)] --> len = 3
				if (len(legsDict[currentLegIndex]['shapePoints']) - 1 > nextShapePtIndex):
					# Yes, there's another shape point.
					# Snap to the shape point we were just aiming for:
					newLatRad = nextLatRad
					newLongRad = nextLongRad
				
					nextShapePtIndex += 1

					nextLatRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][0]*(pi/180)
					nextLongRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][1]*(pi/180)
				
				elif (len(legsDict) - 2 > currentLegIndex):
					# We finished the previous leg, but we haven't run out of legs.
					# NOTE:  The last "leg" doesn't contain any shape points, and it has a distance of 0. 
					# Thus, we have "-2" in our condition.
					currentLegIndex += 1
					nextShapePtIndex = 1  # The first shape point of this leg is equal to the last shape point of the previous leg.  We're interested in the "next" shape point (index 1).
				
					# Snap to the first shape point in the leg
					newLatRad = legsDict[currentLegIndex]['shapePoints'][0][0]*(pi/180)
					newLongRad = legsDict[currentLegIndex]['shapePoints'][0][1]*(pi/180)
				
					nextLatRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][0]*(pi/180)
					nextLongRad = legsDict[currentLegIndex]['shapePoints'][nextShapePtIndex][1]*(pi/180)
				else:
					# We've run out of legs and shape points.  We must've reached our final goal.
					# FIXME -- IS THIS A SAFE ASSUMPTION??
					newLatDeg = goalLatDeg
					newLongDeg = goalLongDeg
					pleaseContinue = 0

			else:
				# Find the angular distance, d/R, where d is the distance that can be traveled 
				# in the remainder of this time interval and R is the earth's radius	
				delta = distanceWeCanTravel/float(radius_of_earth)	
	
				# What angle is required to travel directly from the current location to the next location?
				#	 See http://www.movable-type.co.uk/scripts/latlong.html
				y = sin(nextLongRad - initLongRad) * cos(nextLatRad)
				x = cos(initLatRad)*sin(nextLatRad) - sin(initLatRad)*cos(nextLatRad)*cos(nextLongRad-initLongRad)
				theta = (atan2(y, x) + 2*pi) % (2*pi)	# In the range [0,2*pi]
		
				# Update the new location	
				newLatRad = asin( sin(newLatRad)*cos(delta) + cos(newLatRad)*sin(delta)*cos(theta) )
				newLongRad = newLongRad + atan2( sin(theta)*sin(delta)*cos(newLatRad), cos(delta) - sin(newLatRad)*sin(newLatRad) )
	
				# How long did it take us to get to this new location?
				timeConsumed = distanceWeCanTravel/effSpeed
		
				# Update our remaining time:
				timeRemain -= timeConsumed
			
				print "timeRemain is now %f" % timeRemain
		
			if (timeRemain <= 0.0001):
				pleaseContinue = 0

	
		# Calculate some summary stats:
		distanceToGoal = 0.0
		timeToGoal = 0.0
	
		# a) Loop over shape points to get to the end of the current leg
		for i in range(nextShapePtIndex, len(legsDict[currentLegIndex]['shapePoints'])):
		
			tmpDistance = groundDistanceStraight(newLatRad, newLongRad, legsDict[currentLegIndex]['shapePoints'][i][0]*(pi/180), legsDict[currentLegIndex]['shapePoints'][i][1]*(pi/180))
			distanceToGoal += tmpDistance
			if (legsDict[currentLegIndex]['time'] > 0):
				effSpeed = (legsDict[currentLegIndex]['distance']*METERS_PER_MILE)/float(legsDict[currentLegIndex]['time'])
				timeToGoal += tmpDistance/effSpeed	
			else:
				# We have no distance to travel
				timeToGoal += 0.0

		
		# b) Use info from legsDict to determine total time and distance:
		for i in range(currentLegIndex+1, len(legsDict)-1):
			distanceToGoal += legsDict[i]['distance']*METERS_PER_MILE
			timeToGoal += legsDict[i]['time']
	
		# Go ahead and place the asset at the goal if we're "close-enough":
		if (distanceToGoal <= DIST_TOL):
			newLatRad = goalLatRad
			newLongRad = goalLongRad
		

		newLatDeg = newLatRad*(180/pi)
		newLongDeg = newLongRad*(180/pi)

		return [newLatDeg, newLongDeg, distanceToGoal, timeToGoal, timeRemain, currentLegIndex, nextShapePtIndex]

		
	
def moveMultirotorAsset(takeoffSpeed, cruiseSpeed, landSpeed, yawRateDeg, initAlt, flightAlt, goalAlt, initLatDeg, initLongDeg, goalLatDeg, goalLongDeg, initHeadingDeg, goalHeadingDeg, availTime):

	# Example:
	# moveMultirotorAsset(20.0, 30.0, 10.0, 15.0, 0.0, 50.0, 0.0, 45.0, 45.0, 55.0, 55.0, -45.0, -361, 20.0)
	
	# Returns:
	# 	[newLatDeg, newLongDeg, newHeadingDeg, newAlt, distanceToGoal, timeToGoal, availTimeUnused]
		
	# NOTES:
	#	* lat/long values are in units of [DEGREES]
	#	* altitude values are in units of [meters]
	#	* turn radius is in units of [meters]
	#	* yaw rate is in units of [DEGREES/second]
	#	* heading is in units of [DEGREES]
	#		- If initHeadingDeg == -361 we probably have an error.  WE REALLY NEED TO KNOW THE INITIAL HEADING.
	#		- If goalHeadingDeg == -361, we'll ignore any rotation required at the destination
	#	* availTime is in units of [seconds]
	#	FIXME ... am I missing anything?
	
	if (initHeadingDeg <= -361):
		# We really need to have a valid heading for this to make any sense.
		print "We really need to have a valid heading for this to make any sense."
		print "We're going to assume that the initial heading is 0 degrees."
		initHeadingDeg = 0.0
	
	# Convert units to radians:
	yawRateRad = yawRateDeg*(pi/180)
	initLatRad = initLatDeg*(pi/180)
	initLongRad = initLongDeg*(pi/180)
	goalLatRad = goalLatDeg*(pi/180)
	goalLongRad = goalLongDeg*(pi/180)
	initHeadingRad = initHeadingDeg*(pi/180)
	goalHeadingRad = goalHeadingDeg*(pi/180)
	
	myTime = 0.0
	timeRemain = availTime
	
	# NOTE: It's possible that our initial and goal locations are the same. 
	# In this case, we don't need any travel (esp. takeoff/landing time)
	if (([initLatRad, initLongRad, initAlt] == [goalLatRad, goalLongRad, goalAlt]) and (goalHeadingDeg == -361)):
		# 	[newLatDeg, newLongDeg, newHeadingDeg, newAlt, distanceToGoal, timeToGoal, availTimeUnused]
		return [initLatDeg, initLongDeg, initHeadingDeg, initAlt, 0.0, 0.0, timeRemain]
		
	else:
		newLatRad = initLatRad
		newLongRad = initLongRad
		newHeadingRad = initHeadingRad
		newAlt = initAlt
		
		# 1) Adjust altitude for takeoff 
		#	 NOTE: Only do this if our init/goal coordinates differ (otherwise we're not prepping to fly anywhere)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			deltaAlt = flightAlt - initAlt
			
			timeReqd = (abs(deltaAlt)/takeoffSpeed)
			timeConsumed = min(timeRemain, timeReqd)
			
			if (deltaAlt > 0):
				# we need to go up
				newAlt += takeoffSpeed*timeConsumed
			else:
				# we need to go down
				newAlt -= takeoffSpeed*timeConsumed
				
			timeRemain -= timeConsumed
			 		
		# 2) Rotate towards target (yaw)
		#	 NOTE: Only do this if our init/goal coordinates differ (otherwise we're not prepping to fly anywhere)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			if (timeRemain > 0):
				# What angle is required to travel directly from the current location to the goal location?
				#	 See http://www.movable-type.co.uk/scripts/latlong.html
				y = sin(goalLongRad - newLongRad) * cos(goalLatRad)
				x = cos(newLatRad)*sin(goalLatRad) - sin(newLatRad)*cos(goalLatRad)*cos(goalLongRad-newLongRad)
				headingStrRad = (atan2(y, x) + 2*pi) % (2*pi)	# In the range [0,2*pi]

				# What is the difference between the current heading and the straight-line bearing?
				# The following formula tells us how far we'd have to rotate (in radians) if we rotate CCW:
				deltaHeadingRad = (newHeadingRad - headingStrRad) % (2*pi)

				# Should we rotate CW or CCW?
				if (deltaHeadingRad <= pi):
					# The shortest rotation will be CCW.
					rotateCCW = 1		# Set to true
				else:
					# We'd prefer to rotate CW instead.
					rotateCCW = 0		# Set to false
					# When rotating CW, the actual required rotation angle is less than pi radians:
					deltaHeadingRad = 2*pi - deltaHeadingRad
			
				timeReqd = deltaHeadingRad/yawRateRad
				timeConsumed = min(timeRemain, timeReqd)

				# How many radians can we actually rotate thru (given our time limit)?
				radians = yawRateRad*timeConsumed

				# Rotate this number of radians:
				if (rotateCCW == 1):		
					if (radians > newHeadingRad):
						# We're crossing north CCW from east to west
						newHeadingRad = 2*pi - (radians - newHeadingRad)
					else:
						newHeadingRad = newHeadingRad - radians
				elif (rotateCCW == 0):
					if (newHeadingRad + radians > 2*pi):
						# We're crossing north CW from west to east
						newHeadingRad = 0 + radians - (2*pi - newHeadingRad)
					else:
						newHeadingRad = newHeadingRad + radians
							
				timeRemain -= timeConsumed
			
	
		# 3) Fly to target
		#	 NOTE: Only do this if our init/goal coordinates differ (otherwise we're not flying anywhere)
		if ([initLatRad, initLongRad] != [goalLatRad, goalLongRad]):
			if (timeRemain > 0):
				# How far can we travel in the time remaining?
				distanceToTravel = min(groundDistanceStraight(newLatRad, newLongRad, goalLatRad, goalLongRad), cruiseSpeed*timeRemain)
			
				delta = distanceToTravel/float(radius_of_earth)	# angular distance d/R, where d is the distance that can be traveled in the remainder of this time interval and R is the earth's radius
			
				theta = newHeadingRad		# We should be headed straight towards the target now
			
				# Update the new location	
				newLatRad = asin( sin(newLatRad)*cos(delta) + cos(newLatRad)*sin(delta)*cos(theta) )
				newLongRad = newLongRad + atan2( sin(theta)*sin(delta)*cos(newLatRad), cos(delta) - sin(newLatRad)*sin(newLatRad) )
			
				timeConsumed = distanceToTravel/cruiseSpeed
				timeRemain -= timeConsumed

		
		# 4) Rotate at target to desired heading (if applicable)
		if (timeRemain > 0):
		
			if (goalHeadingDeg <= -361):
				# We don't care about the goal heading
				timeRemain -= 0.0
			else:
				# What is the difference between the current heading and the desired heading?
				# The following formula tells us how far we'd have to rotate (in radians) if we rotate CCW:
				deltaHeadingRad = (newHeadingRad - goalHeadingRad) % (2*pi)

				# Should we rotate CW or CCW?
				if (deltaHeadingRad <= pi):
					# The shortest rotation will be CCW.
					rotateCCW = 1		# Set to true
				else:
					# We'd prefer to rotate CW instead.
					rotateCCW = 0		# Set to false
					# When rotating CW, the actual required rotation angle is less than pi radians:
					deltaHeadingRad = 2*pi - deltaHeadingRad
	
				timeReqd = deltaHeadingRad/yawRateRad
				timeConsumed = min(timeRemain, timeReqd)

				# How many radians can we actually rotate thru (given our time limit)?
				radians = yawRateRad*timeConsumed

				# Rotate this number of radians:
				if (rotateCCW == 1):		
					if (radians > newHeadingRad):
						# We're crossing north CCW from east to west
						newHeadingRad = 2*pi - (radians - newHeadingRad)
					else:
						newHeadingRad = newHeadingRad - radians
				elif (rotateCCW == 0):
					if (newHeadingRad + radians > 2*pi):
						# We're crossing north CW from west to east
						newHeadingRad = 0 + radians - (2*pi - newHeadingRad)
					else:
						newHeadingRad = newHeadingRad + radians
							
				timeRemain -= timeConsumed
				
	
		# 5) Adjust altitude (e.g., Land)
		if (timeRemain > 0):
			deltaAlt = newAlt - goalAlt

			timeReqd = (abs(deltaAlt)/landSpeed)
			timeConsumed = min(timeRemain, timeReqd)

			if (deltaAlt > 0):
				# we need to go down
				newAlt -= landSpeed*timeConsumed
			else:
				# we need to go up
				newAlt += landSpeed*timeConsumed
				
			timeRemain -= timeConsumed
			

	newLatDeg = newLatRad*(180/pi)
	newLongDeg = newLongRad*(180/pi)
	newHeadingDeg = newHeadingRad*(180/pi)
	
	distanceToGoal = groundDistanceStraight(newLatRad, newLongRad, goalLatRad, goalLongRad)

	print distanceToGoal

	if (distanceToGoal <= DIST_TOL):
		newLatDeg = goalLatDeg
		newLongDeg = goalLongDeg

	timeToGoal = calcMultirotorTravelTime(takeoffSpeed, cruiseSpeed, landSpeed, yawRateDeg, newAlt, flightAlt, goalAlt, newLatDeg, newLongDeg, goalLatDeg, goalLongDeg, newHeadingDeg, goalHeadingDeg)

	
	
	return [newLatDeg, newLongDeg, newHeadingDeg, newAlt, distanceToGoal, timeToGoal, timeRemain]


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
	#	FIXME ... am I missing anything?
	
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
