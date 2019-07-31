#!/usr/bin/env python

import sys
import time
import datetime
import math
from gurobipy import *
from collections import defaultdict
from dfs import *

import os


NODE_TYPE_DEPOT		= 0
NODE_TYPE_CUST		= 1

TYPE_TRUCK 			= 1
TYPE_UAV 			= 2

TRAVEL_UAV_PACKAGE		= 1
TRAVEL_UAV_EMPTY		= 2
TRAVEL_TRUCK_W_UAV		= 3
TRAVEL_TRUCK_EMPTY		= 4

VERTICAL_UAV_EMPTY		= 5
VERTICAL_UAV_PACKAGE	= 6

STATIONARY_UAV_EMPTY	= 7
STATIONARY_UAV_PACKAGE	= 8
STATIONARY_TRUCK_W_UAV	= 9
STATIONARY_TRUCK_EMPTY	= 10

GANTT_IDLE		= 1
GANTT_TRAVEL	= 2
GANTT_DELIVER	= 3
GANTT_RECOVER	= 4
GANTT_LAUNCH	= 5
GANTT_FINISHED	= 6

# There's a package color that corresponds to the VEHICLE that delivered the package.
packageIcons		= ['', 'box_blue_centered.gltf', 'box_orange_centered.gltf', 'box_green_centered.gltf', 'box_gray_centered.gltf', 'box_brown_centered.gltf']
# =============================================================



# http://stackoverflow.com/questions/635483/what-is-the-best-way-to-implement-nested-dictionaries-in-python
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

class make_assignments:
	def __init__(self, vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus):
		# Set assignments[v][statusID][statusIndex]
		self.vehicleType 	= vehicleType
		self.startTime 		= startTime
		self.startNodeID	= startNodeID
		self.startLatDeg 	= startLatDeg
		self.startLonDeg 	= startLonDeg
		self.startAltMeters = startAltMeters
		self.endTime 		= endTime
		self.endNodeID 		= endNodeID
		self.endLatDeg		= endLatDeg
		self.endLonDeg		= endLonDeg
		self.endAltMeters 	= endAltMeters
		self.icon			= icon
		self.description 	= description
		self.UAVsOnBoard 	= UAVsOnBoard
		self.ganttStatus	= ganttStatus

class make_packages:
	def __init__(self, packageType, latDeg, lonDeg, deliveryTime, icon):
		# Set packages[nodeID]
		self.packageType 	= packageType
		self.latDeg 		= latDeg
		self.lonDeg 		= lonDeg
		self.deliveryTime 	= deliveryTime
		self.icon 			= icon

def solve_tsp_callback(node, vehicle, travel):
	
	# We want to return this collection of assignments and packages:
	assignments	= defaultdict(make_dict)
	packages	= defaultdict(make_dict)

	# Establish system parameters:
	C 			= []
	N			= []
	N_zero		= []
	N_plus		= []
	tau			= defaultdict(make_dict)
	sigma		= {}

	c = 0
	N.append(0)	# Add the depot
	N_zero.append(0)
	for nodeID in node:
		if (node[nodeID].nodeType == NODE_TYPE_CUST):
			C.append(nodeID)		# C is the vector of customer nodes.  C = [1, 2, ..., c]
			N.append(nodeID)
			N_zero.append(nodeID)
			N_plus.append(nodeID)
			if (nodeID > c):
				c = nodeID

	N.append(c+1)
	N_plus.append(c+1)

	# We need to define node c+1, which is a copy of the depot.
	#print node
	node[c+1] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address)

	# Build the customer service times:
	for k in N_plus:
		if (k == c+1):
			sigma[k] = 0.0
		else:
			sigma[k] = node[k].serviceTimeTruck

	cost = {}
	# Build tau (truck travel time)
	for vehicleID in vehicle:
		for i in N_zero:
			for j in C:
				if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
					tau[i][j] = travel[vehicleID][i][j].totalTime
					cost[i,j] = travel[vehicleID][i][j].totalTime

			# NOTE: We need to capture the travel time to node c+1 (which is the same physical location as node 0):
			if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
				tau[i][c+1] = travel[vehicleID][i][0].totalTime
				cost[i,0] = travel[vehicleID][i][0].totalTime


	# Callback - use lazy constraints to eliminate sub-tours
	def subtourelim(model, where):
		# this tells Gurobi to stop when it finds an integer solution
		if where == GRB.Callback.MIPSOL:
			# make a list of edges selected in the solution

			delta_dfs = defaultdict(list)

			for i in N_zero:
				for j in N_zero:
					if i != j:
						x_sol = model.cbGetSolution(x[i,j])
						if x_sol > 0.5:
							delta_dfs[i].append(j)
							delta_dfs[j].append(i)

			# Here goes your DFS code for finding connected components
			# use edges to construct the adjacency lists
			component = dfs(N_zero, delta_dfs)

			if (len(component) > 1):
				# add subtour elimination constraints for every pair of nodes in the
				# connected components you found. For each component add the cut
				for k in component:
					model.cbLazy(quicksum((quicksum(x[i,j] for j in component[k] if j != i)) for i in component[k]) <= len(component[k]) - 1)

	# Generate a solution via DFJ - Callback:
	m = Model("dfj_callback")

	# Tell Gurobi not to print to a log file
	m.params.OutputFlag = 0

	# Create variables:
	x = {}

	for i in N_zero:
		for j in N_zero:
			if i != j:
				x[i,j] = m.addVar(lb=0, obj=float(cost[i,j]), vtype=GRB.BINARY, name="x.%f.%f" % (i,j))

	m.modelSense = GRB.MINIMIZE

	m.Params.timeLimit = 600

	m.update()

	# add constraints
	for i in N_zero:
		m.addConstr(quicksum(x[i,j] for j in N_zero if j != i) == 1, "Constr.1.%f" % (i))

	for j in N_zero:
		m.addConstr(quicksum(x[i,j] for i in N_zero if i != j) == 1, "Constr.2.%f" % (j))

	m.Params.lazyConstraints = 1
	m.optimize(subtourelim)


	myTour = [0]

	tmp_i = 0

	for tmploop in range(1,len(N_zero)):
		for tmp_j in N_zero:
			if tmp_i != tmp_j:
				if x[tmp_i,tmp_j].x > 0.9:
					myTour.append(tmp_j)
					tmp_i = tmp_j
					break

	myTour.append(c+1)
	
	# Build the assignment
	vehicleType = TYPE_TRUCK
	UAVsOnBoard = []
	startAltMeters = 0.0
	endAltMeters = 0.0

	i = 0					# Start at the depot
	mayEnd = 0
	tmpDepart = 0.0
	icon = 'ub_truck_1.gltf'

	for myIndex in range(1,len(myTour)):
		j = myTour[myIndex]
		# We are traveling from i to j
		# Capture the "traveling" component:
		statusID 	= TRAVEL_TRUCK_EMPTY
		ganttStatus	= GANTT_TRAVEL
		startTime 	= tmpDepart	# When we departed from i
		startNodeID = i
		startLatDeg = node[i].latDeg
		startLonDeg	= node[i].lonDeg
		endTime		= startTime + tau[i][j]	# This is when we arrive at j
		endNodeID	= j
		endLatDeg	= node[j].latDeg
		endLonDeg	= node[j].lonDeg
		if ((i in C) and (j in C)):
			description 	= 'Driving from Customer %d to Customer %d' % (i,j)
		elif ((i == 0) and (j in C)):
			description 	= 'Driving from Depot to Customer %d' % (j)
		elif ((i in C) and (j == c+1)):
			description 	= 'Returning to the Depot from Customer %d' % (i)
		elif ((i == 0) and (j == c+1)):
			description 	= 'Truck 1 was not used'
		else:
			print('WE HAVE A PROBLEM.  What is the proper description?')
			print('\t Quitting Now.')
			exit()

		if (0 in assignments[1][statusID]):
			statusIndex = len(assignments[1][statusID])
		else:
			statusIndex = 0

		assignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		# Now, capture the "service" component:
		startTime 		= endTime		# When we arrived at j
		startNodeID 	= j
		startLatDeg 	= node[j].latDeg
		startLonDeg		= node[j].lonDeg
		endTime			= startTime + sigma[j]	# This is when we finish up at j
		endNodeID		= j
		endLatDeg		= node[j].latDeg
		endLonDeg		= node[j].lonDeg
		objVal 			= endTime
		if (j == c+1):
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_FINISHED
			tmpMin, tmpSec	= divmod(endTime, 60)
			tmpHour, tmpMin = divmod(tmpMin, 60)
			description		= 'Arrived at the Depot.  Total Time = %d:%02d:%02d' % (tmpHour, tmpMin, tmpSec)
			endTime			= -1
		else:
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_DELIVER
			description		= 'Dropping off package to Customer %d' % (j)
			packageType 	= TYPE_TRUCK
			pkgIcon 		= packageIcons[1]
			packages[j] 	= make_packages(packageType, endLatDeg, endLonDeg, endTime, pkgIcon)

		if (0 in assignments[1][statusID]):
			statusIndex = len(assignments[1][statusID])
		else:
			statusIndex = 0

		assignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		tmpDepart = endTime
		if (j != c+1):
			# Go to the next arc
			i = j

	return (objVal, assignments, packages, myTour)