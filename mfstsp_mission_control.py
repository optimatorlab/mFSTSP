#!/usr/bin/env python

# INSTALLING ADDITIONAL PYTHON MODULES:
#
# pandas:
#	sudo pip install pandas
#
# scipy:
#	sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas
#
# 		DID NOT DO THESE:
#			sudo apt-get install python-numpy python-scipy
#			sudo python -m pip install --upgrade pip
#			sudo pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose
#
# geopy
#	sudo pip install geopy
#
# R TSP
# 	R		[NOTE:  This was ono version 3.0.2]
#	> install.packages("TSP")
# 	[agree to install in another location (defaults to /home/murray/R/x86_64-pc-linux-gnu-library/3.0)
#	I chose CA (ON) as the mirror.  Some mirrors don't have the TSP package. 



# RUNNING THIS SCRIPT:

# python mfstsp_mission_control.py <city> <problemName> <vehicleFileID> <cutoffTime> <problemType> <numUAVs> <numTrucks> <requireTruckAtDepot> <requireDriver>

# city: 'buffalo' or 'seattle'
# problemName: Name of the folder containing the data for a particular problem instance
# vehicleFileID: 101, 102, 103, 104 (Chooses a particular UAV type depending on the file ID)
# cutoffTime: Gurobi cut-off time of IP model. In case of heuristic, it is the Gurobi cut-off time of (P3) model
# problemType: 1 (mFSTSP IP) or 2 (mFSTSP Heuristic)
# numUAVs: Number of UAVs available in the problem
# numTrucks: Number of trucks available in the problem (currently only solvable for 1 truck)
# requireTruckAtDepot:  0 (false) or 1 (true).
# requireDriver: 0 (false) or 1 (true). False --> UAVs can launch/land without driver (so driver can serve customer).


# 1) Solving the mFSTSP optimally:
#    python mfstsp_mission_control.py seattle 20170608T121632668184 101 3600 1 3 -1 1 1
#		numTrucks is ignored
#		The UAVs are defined in tbl_vehicles.  If you ask for more UAVs than are defined, you'll get a warning.

# 2) Solving the mFSTSP via a heuristic:
#    python mfstsp_mission_control.py seattle 20170608T121632668184 101 5 2 3 -1 1 1
#		numTrucks is ignored
#		The UAVs are defined in tbl_vehicles.  If you ask for more UAVs than are defined, you'll get a warning.


import sys
import datetime
import time
import math
from collections import defaultdict
import pandas as pd

from parseCSV import *
from parseCSVstring import *

from gurobipy import *
import os
import os.path
from subprocess import call		# allow calling an external command in python.  See http://stackoverflow.com/questions/89228/calling-an-external-command-in-python

import urllib2
import json
import numpy

from solve_mfstsp_IP import *
from solve_mfstsp_heuristic import *

import distance_functions

# =============================================================
startTime 		= time.time()
homeDirectory 	= os.environ['HOME']  # '/home/murray'

METERS_PER_MILE = 1609.34

# PROBLEM_TYPE
# 1 --> mFSTSP optimal
# 2 --> mFSTSP heuristic (will need other parameters)
problemTypeString = {1: 'mFSTSP IP', 2: 'mFSTSP Heuristic'}


NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2

MODE_CAR 		= 1
MODE_BIKE 		= 2
MODE_WALK 		= 3
MODE_FLY 		= 4

ACT_TRAVEL 			= 0
ACT_SERVE_CUSTOMER	= 1
ACT_DONE			= 2

# =============================================================

def make_dict():
	return defaultdict(make_dict)

	# Usage:
	# tau = defaultdict(make_dict)
	# v = 17
	# i = 3
	# j = 12
	# tau[v][i][j] = 44

class make_node:
	def __init__(self, nodeType, latDeg, lonDeg, altMeters, parcelWtLbs, serviceTimeTruck, serviceTimeUAV, address):
		# Set node[nodeID]
		self.nodeType 			= nodeType
		self.latDeg 			= latDeg
		self.lonDeg				= lonDeg
		self.altMeters			= altMeters
		self.parcelWtLbs 		= parcelWtLbs
		self.serviceTimeTruck	= serviceTimeTruck	# [seconds]
		self.serviceTimeUAV 	= serviceTimeUAV	# [seconds]
		self.address 			= address			# Might be None

class make_vehicle:
	def __init__(self, vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange):
		# Set vehicle[vehicleID]
		self.vehicleType	= vehicleType
		self.takeoffSpeed	= takeoffSpeed
		self.cruiseSpeed	= cruiseSpeed
		self.landingSpeed	= landingSpeed
		self.yawRateDeg		= yawRateDeg
		self.cruiseAlt		= cruiseAlt
		self.capacityLbs	= capacityLbs
		self.launchTime		= launchTime	# [seconds].
		self.recoveryTime	= recoveryTime	# [seconds].
		self.serviceTime	= serviceTime
		self.batteryPower	= batteryPower	# [joules].
		self.flightRange	= flightRange	# 'high' or 'low'

class make_travel:
	def __init__(self, takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance):
		# Set travel[vehicleID][fromID][toID]
		self.takeoffTime 	 = takeoffTime
		self.flyTime 		 = flyTime
		self.landTime 		 = landTime
		self.totalTime 		 = totalTime
		self.takeoffDistance = takeoffDistance
		self.flyDistance	 = flyDistance
		self.landDistance	 = landDistance
		self.totalDistance	 = totalDistance


class missionControl():
	def __init__(self):

		timestamp = datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S')

		# python mfstsp_mission_control.py seattle 20170608T121632668184 101 3600 1 3 -1 1 1
		# Capture 9 inputs from the command line
		# NOTE: sys.argv[0] is the name of the python file
		# Try "print sys.argv" (without the quotes) to see the sys.argv list
		# 9 inputs --> the sys.argv list should have 10 elements.
		if (len(sys.argv) == 10):
			city				= sys.argv[1]
			problemName 		= sys.argv[2]
			vehicleFileID		= int(sys.argv[3])			
			cutoffTime 			= float(sys.argv[4])
			problemType 		= int(sys.argv[5])
			numUAVs				= int(sys.argv[6])
			numTrucks			= int(sys.argv[7])
			requireTruckAtDepot = bool(int(sys.argv[8]))
			requireDriver 		= bool(int(sys.argv[9]))
			

			self.locationsFile = 'Problems/%s/tbl_locations.csv' % (problemName)
			self.vehiclesFile = 'Problems/tbl_vehicles_%d.csv' % (vehicleFileID)
			if problemType == 1:
				indicator = 'IP'
			elif problemType == 2:
				indicator = 'Heuristic'
			self.solutionSummaryFile = 'Problems/%s/tbl_solutions_%d_%d_%s.csv' % (problemName, vehicleFileID, numUAVs, indicator)
			self.distmatrixFile = 'Problems/%s/tbl_truck_travel_data_PG.csv' % (problemName)

		else:
			print 'ERROR: You passed', len(sys.argv)-1, 'input parameters.'
			quit()


		# Define data structures
		self.node = {}
		self.vehicle = {}
		self.travel = defaultdict(make_dict)

		# Read data for node locations, vehicle properties, and travel time matrix of truck:
		self.readData(numUAVs)

		# Calculate travel times of UAVs (travel times of truck has already been read when we called the readData function)
		# NOTE:  For each vehicle we're going to get a matrix of travel times from i to j,
		#		 where i is in [0, # of customers] and j is in [0, # of customers].
		#		 However, tau and tauPrime let node c+1 represent a copy of the depot.
		for vehicleID in self.vehicle:
			if (self.vehicle[vehicleID].vehicleType == TYPE_UAV):
				# We have a UAV (Note:  In some problems we only have a truck)
				for i in self.node:
					for j in self.node:
						if (j == i):
							self.travel[vehicleID][i][j] = make_travel(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
						else:
							[takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance] = distance_functions.calcMultirotorTravelTime(self.vehicle[vehicleID].takeoffSpeed, self.vehicle[vehicleID].cruiseSpeed, self.vehicle[vehicleID].landingSpeed, self.vehicle[vehicleID].yawRateDeg, self.node[i].altMeters, self.vehicle[vehicleID].cruiseAlt, self.node[j].altMeters, self.node[i].latDeg, self.node[i].lonDeg, self.node[j].latDeg, self.node[j].lonDeg, -361, -361)
							self.travel[vehicleID][i][j] = make_travel(takeoffTime, flyTime, landTime, totalTime, takeoffDistance, flyDistance, landDistance, totalDistance)


		# Now, call the IP / Heuristic model:
		isOptimal = False
		if (problemType == 1):
			print 'Calling Gurobi to solve mFSTSP...'
			[objVal, assignments, packages, isOptimal, bestBound, waitingTruck, waitingUAV] = solve_mfstsp_IP(self.node, self.vehicle, self.travel, cutoffTime, requireTruckAtDepot, requireDriver)
			print 'Gubori is Done.  It returned something'
		elif (problemType == 2):
			print 'Calling a Heuristic to solve mFSTSP...'
			[objVal, assignments, packages, waitingTruck, waitingUAV] = solve_mfstsp_heuristic(self.node, self.vehicle, self.travel, cutoffTime, problemName, problemType, requireTruckAtDepot, requireDriver)
			bestBound = -1
			print 'The mFSTSP Heuristic is Done.  It returned something'
		else:
			print 'Sorry, I do not understand problemType = %d.  Bye.' % (problemType)


		numUAVcust			= 0
		numTruckCust		= 0		
		for nodeID in packages:
			if (self.node[nodeID].nodeType == NODE_TYPE_CUST):
				if (packages[nodeID].packageType == TYPE_UAV):
					numUAVcust		+= 1
				else:
					numTruckCust	+= 1

			
		# Write in the performance_summary file:
		total_time = time.time() - startTime
		print "Total time for the whole process: %f" % (total_time)
		print "Objective Function Value: %f" % (objVal)

		runString = ' '.join(sys.argv[0:])

		myFile = open('performance_summary.csv','a')
		str = '%s, %d, %f, %d, %s, %d, %d, %s, %s, %s,' % (problemName, vehicleFileID, cutoffTime, problemType, problemTypeString[problemType], numUAVs, numTrucks, requireTruckAtDepot, requireDriver, runString)
		myFile.write(str)

		numCustomers = len(self.node) - 2
		str = '%d, %s, %f, %f, %f, %s, %d, %d, %f, %f \n' % (numCustomers, timestamp, objVal, bestBound, total_time, isOptimal, numUAVcust, numTruckCust, waitingTruck, waitingUAV)
		myFile.write(str)
					
		myFile.close()
		print "\nSee 'performance_summary.csv' for statistics.\n"


		# Write in the solution file:
		myFile = open(self.solutionSummaryFile, 'a')
		myFile.write('problemName, vehicleFileID, cutoffTime, problemTypeString, numUAVs, numTrucks, requireTruckAtDepot, requireDriver \n')
		str = '%s, %d, %f, %s, %d, %d, %s, %s \n\n' % (problemName, vehicleFileID, cutoffTime, problemTypeString[problemType], numUAVs, numTrucks, requireTruckAtDepot, requireDriver)
		myFile.write(str)

		myFile.write('Objective Function Value: %f \n\n' % (objVal))
		myFile.write('Assignments: \n')

		myFile.close()

		# Create a dataframe to sort assignments according to their start times:
		assignDF = pd.DataFrame(columns=['vehicleID', 'vehicleType', 'activityType', 'startTime', 'startNode', 'endTime', 'endNode', 'Description', 'Status'])
		indexDF = 1

		for v in assignments:
			for statusID in assignments[v]:
				for statusIndex in assignments[v][statusID]:
					if (assignments[v][statusID][statusIndex].vehicleType == TYPE_TRUCK):
						vehicleType = 'Truck'
					else:
						vehicleType = 'UAV'
					if (statusID == TRAVEL_UAV_PACKAGE):
						status = 'UAV travels with parcel'
					elif (statusID == TRAVEL_UAV_EMPTY):
						status = 'UAV travels empty'
					elif (statusID == TRAVEL_TRUCK_W_UAV):
						status = 'Truck travels with UAV(s) on board'
					elif (statusID == TRAVEL_TRUCK_EMPTY):
						status = 'Truck travels with no UAVs on board'
					elif (statusID == VERTICAL_UAV_EMPTY):
						status = 'UAV taking off or landing with no parcels'
					elif (statusID == VERTICAL_UAV_PACKAGE):
						status = 'UAV taking off or landing with a parcel'
					elif (statusID == STATIONARY_UAV_EMPTY):
						status = 'UAV is stationary without a parcel'
					elif (statusID == STATIONARY_UAV_PACKAGE):
						status = 'UAV is stationary with a parcel'
					elif (statusID == STATIONARY_TRUCK_W_UAV):
						status = 'Truck is stationary with UAV(s) on board'
					elif (statusID == STATIONARY_TRUCK_EMPTY):
						status = 'Truck is stationary with no UAVs on board'
					else:
						print 'UNKNOWN statusID.'
						quit()

					
					if (assignments[v][statusID][statusIndex].ganttStatus == GANTT_IDLE):
						ganttStr = 'Idle'
					elif (assignments[v][statusID][statusIndex].ganttStatus == GANTT_TRAVEL):
						ganttStr = 'Traveling'
					elif (assignments[v][statusID][statusIndex].ganttStatus == GANTT_DELIVER):
						ganttStr = 'Making Delivery'
					elif (assignments[v][statusID][statusIndex].ganttStatus == GANTT_RECOVER):
						ganttStr = 'UAV Recovery'
					elif (assignments[v][statusID][statusIndex].ganttStatus == GANTT_LAUNCH):
						ganttStr = 'UAV Launch'
					elif (assignments[v][statusID][statusIndex].ganttStatus == GANTT_FINISHED):
						ganttStr = 'Vehicle Tasks Complete'
					else:
						print 'UNKNOWN ganttStatus'
						quit()
					
					assignDF.loc[indexDF] = [v, vehicleType, status, assignments[v][statusID][statusIndex].startTime, assignments[v][statusID][statusIndex].startNodeID, assignments[v][statusID][statusIndex].endTime, assignments[v][statusID][statusIndex].endNodeID, assignments[v][statusID][statusIndex].description, ganttStr]	
					indexDF += 1
		
		assignDF = assignDF.sort_values(by=['vehicleID', 'startTime'])

		# Add this assignment dataframe to the solution file:
		assignDF.to_csv(self.solutionSummaryFile, mode='a', header=True, index=False)
		
		print "\nSee '%s' for solution summary.\n" % (self.solutionSummaryFile)
		


	def readData(self, numUAVs):
		# b)  tbl_vehicles.csv
		tmpUAVs = 0
		rawData = parseCSVstring(self.vehiclesFile, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%')
		for i in range(0,len(rawData)):
			vehicleID 			= int(rawData[i][0])
			vehicleType			= int(rawData[i][1])
			takeoffSpeed		= float(rawData[i][2])
			cruiseSpeed			= float(rawData[i][3])
			landingSpeed		= float(rawData[i][4])
			yawRateDeg			= float(rawData[i][5])
			cruiseAlt			= float(rawData[i][6])
			capacityLbs			= float(rawData[i][7])
			launchTime			= float(rawData[i][8])	# [seconds].
			recoveryTime		= float(rawData[i][9])	# [seconds].
			serviceTime			= float(rawData[i][10])	# [seconds].
			batteryPower		= float(rawData[i][11])	# [joules].
			flightRange			= str(rawData[i][12])	# 'high' or 'low'
			
			if (vehicleType == TYPE_UAV):
				tmpUAVs += 1
				if (tmpUAVs <= numUAVs):
					self.vehicle[vehicleID] = make_vehicle(vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange)
			else:
				self.vehicle[vehicleID] = make_vehicle(vehicleType, takeoffSpeed, cruiseSpeed, landingSpeed, yawRateDeg, cruiseAlt, capacityLbs, launchTime, recoveryTime, serviceTime, batteryPower, flightRange)

		if (tmpUAVs < numUAVs):
			print "WARNING: You requested %d UAVs, but we only have data on %d UAVs." % (numUAVs, tmpUAVs)
			print "\t We'll solve the problem with %d UAVs.  Sorry." % (tmpUAVs)

		# a)  tbl_locations.csv
		rawData = parseCSVstring(self.locationsFile, returnJagged=False, fillerValue=-1, delimiter=',', commentChar='%')
		for i in range(0,len(rawData)):
			nodeID 				= int(rawData[i][0])
			nodeType			= int(rawData[i][1])
			latDeg				= float(rawData[i][2])		# IN DEGREES
			lonDeg				= float(rawData[i][3])		# IN DEGREES
			altMeters			= float(rawData[i][4])
			parcelWtLbs			= float(rawData[i][5])
			if (len(rawData[i]) == 10):
				addressStreet	= str(rawData[i][6])
				addressCity		= str(rawData[i][7])
				addressST		= str(rawData[i][8])
				addressZip		= str(rawData[i][9])
				address			= '%s, %s, %s, %s' % (addressStreet, addressCity, addressST, addressZip)
			else:
				address 		= '' # or None?

			serviceTimeTruck	= self.vehicle[1].serviceTime
			if numUAVs > 0:
				serviceTimeUAV	= self.vehicle[2].serviceTime
			else:
				serviceTimeUAV = 0

			self.node[nodeID] = make_node(nodeType, latDeg, lonDeg, altMeters, parcelWtLbs, serviceTimeTruck, serviceTimeUAV, address)

		# c) tbl_truck_travel_data.csv
		if (os.path.isfile(self.distmatrixFile)):
			# Travel matrix file exists
			rawData = parseCSV(self.distmatrixFile, returnJagged=False, fillerValue=-1, delimiter=',')
			for i in range(0,len(rawData)):
				tmpi 	= int(rawData[i][0])
				tmpj 	= int(rawData[i][1])
				tmpTime	= float(rawData[i][2])
				tmpDist	= float(rawData[i][3])

				for vehicleID in self.vehicle:
					if (self.vehicle[vehicleID].vehicleType == TYPE_TRUCK):
						self.travel[vehicleID][tmpi][tmpj] = make_travel(0.0, tmpTime, 0.0, tmpTime, 0.0, tmpDist, 0.0, tmpDist)

		else:
			# Travel matrix file does not exist
			print "ERROR: Truck travel data is not available. Please provide a data matrix in the following format in a CSV file, and try again:\n"
			print "from node ID | to node ID | travel time [seconds] | travel distance [meters]\n"
			exit()


if __name__ == '__main__':
	try:
		missionControl()
	except:
		print "There was a problem.  Sorry things didn't work out.  Bye."
		raise