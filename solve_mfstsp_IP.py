#!/usr/bin/env python

import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from collections import defaultdict

import endurance_calculator
import distance_functions


# =============================================================
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

METERS_PER_MILE = 1609.34


# There's a package color that corresponds to the VEHICLE that delivered the package.
# Right now we only have 5 boxes (so we can have at most 5 trucks).
packageIcons		= ['box_yellow_centered.gltf', 'box_blue_centered.gltf', 'box_orange_centered.gltf', 'box_green_centered.gltf', 'box_gray_centered.gltf', 'box_brown_centered.gltf']
# =============================================================


# http://stackoverflow.com/questions/635483/what-is-the-best-way-to-implement-nested-dictionaries-in-python
def make_dict():
	return defaultdict(make_dict)



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
		self.address 			= address			# Might be None...need MapQuest to give us this info later.

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


def solve_mfstsp_IP(node, vehicle, travel, cutoffTime, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, Etype):
	
	# Establish Gurobi data sets
	C 			= []
	tau			= defaultdict(make_dict)
	tauprime 	= defaultdict(make_dict)
	eee 		= defaultdict(make_dict)
	V			= []		# Set of UAVs.
	sL			= defaultdict(make_dict)
	sR			= defaultdict(make_dict)
	sigma		= {}
	sigmaprime	= {}


	for nodeID in node:	
		if (node[nodeID].nodeType == NODE_TYPE_CUST):
			C.append(nodeID)		# C is the vector of customer nodes.  C = [1, 2, ..., c]
			
			
	for vehicleID in vehicle:
		if (vehicle[vehicleID].vehicleType == TYPE_UAV):
			V.append(vehicleID)		# V is the vector of vehicles
												
	c = len(C)				# c is the number of customers
	N = range(0,c+2)		# N = [0, 1, 2, ..., c+1]
	N_zero = range(0,c+1)	# N_zero = [0, 1, ..., c]
	N_plus = range(1,c+2)	# N_plus = [1, 2, ..., c+1]
	
	
	# We need to define node c+1, which is a copy of the depot.
	node[c+1] = make_node(node[0].nodeType, node[0].latDeg, node[0].lonDeg, node[0].altMeters, node[0].parcelWtLbs, node[0].serviceTimeTruck, node[0].serviceTimeUAV, node[0].address) 
	
	
	# Build tau (truck) and tauprime (UAV):
	minDistance = 0			# We'll use this to calculate big M later
	for vehicleID in vehicle:
		for i in N_zero:
			for j in N_zero:
				if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
					tau[i][j] = travel[vehicleID][i][j].totalTime
					if (tau[i][j] > minDistance):
						minDistance = tau[i][j]
				elif (vehicle[vehicleID].vehicleType == TYPE_UAV):
					tauprime[vehicleID][i][j] = travel[vehicleID][i][j].totalTime
				else:
					print("ERROR:  Vehicle Type %d is not defined." % (vehicle[vehicleID].vehicleType))
					quit()	
					
			# NOTE: We need to capture the travel time to node c+1 (which is the same physical location as node 0):
			if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
				tau[i][c+1] = travel[vehicleID][i][0].totalTime
				if (tau[i][c+1] > minDistance):
					minDistance = tau[i][c+1]
			elif (vehicle[vehicleID].vehicleType == TYPE_UAV):
				tauprime[vehicleID][i][c+1] = travel[vehicleID][i][0].totalTime
			else:
				print("ERROR:  Vehicle Type %d is not defined." % (vehicle[vehicleID].vehicleType))
				quit()	


	# Build the set of all possible sorties:
	P = []
	for v in vehicle:
		for i in N_zero:
			for j in C:
				if ((j != i) and (node[j].parcelWtLbs <= vehicle[v].capacityLbs)):
					for k in N_plus:
						if (k != i) and (k != j):
							
							# Calculate the endurance for each sortie:
							if (k == c+1):
								eee[v][i][j][k] = endurance_calculator.give_endurance(node, vehicle, travel, v, i, j, 0, Etype)
							else:	
								eee[v][i][j][k] = endurance_calculator.give_endurance(node, vehicle, travel, v, i, j, k, Etype)
							
							# If endurance is based on distance, build the P set using distance limitations:
							if Etype == 5:
								DISTij = distance_functions.groundDistanceStraight(node[i].latDeg*(math.pi/180), node[i].lonDeg*(math.pi/180), node[j].latDeg*(math.pi/180), node[j].lonDeg*(math.pi/180))
								DISTjk = distance_functions.groundDistanceStraight(node[j].latDeg*(math.pi/180), node[j].lonDeg*(math.pi/180), node[k].latDeg*(math.pi/180), node[k].lonDeg*(math.pi/180))

								if vehicle[v].flightRange == 'low':
									if DISTij + DISTjk <= 6*METERS_PER_MILE:
										P.append([v,i,j,k])
								elif vehicle[v].flightRange == 'high':
									if DISTij + DISTjk <= 12*METERS_PER_MILE:
										P.append([v,i,j,k])

							else:
								if (tauprime[v][i][j] + node[j].serviceTimeUAV + tauprime[v][j][k] <= eee[v][i][j][k]):
									P.append([v,i,j,k])


	for v in V:
		# Build the launch service times:
		for i in N_zero:
			sL[v][i] = vehicle[v].launchTime

		# Build the recovery service times:
		for k in N_plus:
			sR[v][k] = vehicle[v].recoveryTime
			


	# Build the customer service times:
	for k in N_plus:
		if (k == c+1):
			sigma[k] = 0.0
			sigmaprime[k] = 0.0
		else:
			sigma[k] = node[k].serviceTimeTruck	
			sigmaprime[k] = node[k].serviceTimeUAV
		

	LTL = math.ceil((float(len(C) - len(V))/float(1 + len(V))))		# This is the actual lower truck limit.
			
	# 2) GUROBI
	# Model
	m = Model("mFSTSP")

	# Tell Gurobi not to print to a log file
	m.params.OutputFlag = 0


	# a) Decision Variable Definitions and Objective Function:
	decvarx 		= defaultdict(make_dict)
	decvary 		= defaultdict(make_dict)
	decvarp 		= defaultdict(make_dict)			
	decvaru 		= defaultdict(make_dict)

	decvarcheckt 	= defaultdict(make_dict)
	decvarbart 		= defaultdict(make_dict)
	decvarhatt 		= defaultdict(make_dict)

	decvarchecktprime 	= defaultdict(make_dict)
	decvarhattprime 	= defaultdict(make_dict)

	decvarzl 		= defaultdict(make_dict)
	decvarzr 		= defaultdict(make_dict)
	decvarzprime 	= defaultdict(make_dict)
	decvarzdp 		= defaultdict(make_dict)

	RELAX = False
	if (RELAX):
		myVtype = GRB.CONTINUOUS
	else:
		myVtype = GRB.BINARY
			
			
	for i in N_zero:
		for j in N_plus:
			if (i != j):
				decvarx[i][j] = m.addVar(lb=0, ub=1, obj=0, vtype=myVtype, name="x.%d.%d" % (i,j))

		for j in C:
			if (j != i):
				if (i == 0):
					# Hard-code p_{0,j} = 1 for all j in C:
					decvarp[i][j] = m.addVar(lb=1, ub=1, obj=0, vtype=myVtype, name="p.%d.%d" % (i,j))
				else:
					decvarp[i][j] = m.addVar(lb=0, ub=1, obj=0, vtype=myVtype, name="p.%d.%d" % (i,j))

	for v in V:
		# UAVs only!
		for i in N_zero:
			for j in C:
				if (j != i):
					for k in N_plus:
						if ([v,i,j,k] in P):
							decvary[v][i][j][k] = m.addVar(lb=0, ub=1, obj=0, vtype=myVtype, name="y.%d.%d.%d.%d" % (v,i,j,k))
		# UAVs only!
		for i in N:
			decvarchecktprime[v][i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checktprime.%d.%d" % (v, i))
			decvarhattprime[v][i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hattprime.%d.%d" % (v, i))

	
	for i in N:
		if (REQUIRE_TRUCK_AT_DEPOT):
			if (i == c+1):
				# This is our only term in the objective function (for hatt):
				decvarhatt[i] = m.addVar(lb = 0, obj=1, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			elif (i == 0):
				# DON'T Hard-code \hat{t}_{0} = \bar{t}_{0} = 0.
				# However, \check{t}_{0} = 0
				decvarhatt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))			
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			else:
				# Just a regular non-negative continuous decision variable:
				decvarhatt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
		else:
			if (i == c+1):
				# This is our only term in the objective function (for hatt):
				decvarhatt[i] = m.addVar(lb = 0, obj=1, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			elif (i == 0):
				# Hard-code \hat{t}_{0} = \check{t}_{0} = \bar{t}_{0} = 0:
				decvarhatt[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))			
				decvarbart[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			else:
				# Just a regular non-negative continuous decision variable:
				decvarhatt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			


	if (REQUIRE_DRIVER):
		# We'll need to define zr[v][0][k], zr[0][v][k], zl[v][0][k], and zl[0][v][k]
		zzz = set().union([0], V)
	else:
		# We don't need to include the truck (index 0) in these variables.
		zzz = V
			
	for v1 in zzz:
		for v2 in zzz:
			if (v2 != v1):
				for k in N_plus:
					if ((v1 in V) and (v2 == 0) and (k == c+1)):
						# Hardcode z^{R}_{v, 0, c+1} = 0 for all v in V:
						decvarzr[v1][v2][k] = m.addVar(lb = 0, ub = 0, obj = 0, vtype=myVtype, name="zr.%d.%d.%d" % (v1,v2,k))
					else:	
						decvarzr[v1][v2][k] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zr.%d.%d.%d" % (v1,v2,k))

				for k in N_zero:
					decvarzl[v1][v2][k] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zl.%d.%d.%d" % (v1,v2,k))						

	for v1 in V:
		for v2 in V:
			if (v2 != v1):
				for k in C:
					decvarzprime[v1][v2][k] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zprime.%d.%d.%d" % (v1,v2,k))
					decvarzdp[v1][v2][k] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zdp.%d.%d.%d" % (v1,v2,k))

	for i in N_plus:
		decvaru[i] = m.addVar(lb=1, ub=c+2, obj=0, vtype=GRB.CONTINUOUS, name="u.%d" % (i))

	
	# Define M
	M = 0		# Initialize
	unvisitedCustomers = range(1,c+1)   # We haven't visited anyone yet.	
	i = 0		# Start at the depot

	while (len(unvisitedCustomers) > 0):
		# Find the nearest customer.
		# Break ties by selecting the customer with the smallest node number
		
		# minDistance = max(max(tau))		# Initialize to a big value  
		# We found this value earlier
		tmpMinDist = minDistance
		for j in unvisitedCustomers:
			if (tau[i][j] <= tmpMinDist):
				tmpMinDist = tau[i][j]
				jstar = j
				
		M += tau[i][jstar] + node[jstar].serviceTimeTruck
		unvisitedCustomers.remove(jstar)
		i = jstar
	
	# Route back to the depot:
	M += tau[i][c+1]

		
	# The objective is to minimize the total travel distance.
	m.modelSense = GRB.MINIMIZE
		
	# Give Gurobi a time limit
	if (cutoffTime > 0):
		m.params.TimeLimit = cutoffTime
	
	# Update model to integrate new variables
	m.update()

	#### Start adding constraints:

	# Constraint ensuring that there are a minimum of LTL number of truck customers: (NOT IN THE IP MODEL)
	m.addConstr(quicksum(quicksum(decvarx[i][j] for j in C if j != i) for i in N_zero) >= LTL, "MIN.LTL")


	for j in C:
		# Constraint (2): Visit each customer exactly once
		m.addConstr(quicksum(decvarx[i][j] for i in N_zero if i != j) + quicksum(quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for i in N_zero if i != j) for v in V)  == 1, "Constr.2.%d" % j)
		
		# Constraint (5): IN=OUT, truck
		m.addConstr(quicksum(decvarx[i][j] for i in N_zero if i != j) == quicksum(decvarx[j][k] for k in N_plus if k != j), "Constr.5.%d" % j)


	# Constraint (3): Truck must leave the depot
	m.addConstr(quicksum(decvarx[0][j] for j in N_plus) == 1, "Constr.3")

	# Constraint (4): Truck must return to the depot
	m.addConstr(quicksum(decvarx[i][c+1] for i in N_zero) == 1, "Constr.4")

	
	for v in V:
		for i in N_zero:
			# Constraint (6): UAV may be launched from any node (including the depot) at most once
			m.addConstr(quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i) <= 1), "Constr.6.%d.%d" % (v, i)
			
			for k in N_plus:
				if (k!=i):
					for l in C:
						if (l!=i):
							if (l!=k):
								exprl = LinExpr()
								exprl.clear()
								exprr = LinExpr()
								exprr.clear()
					
								exprl.add(decvarhattprime[v][l],1)
								exprl.add(decvarchecktprime[v][k],-1)
								exprr.addConstant(-3*M)
								exprr.add(decvarp[i][l],M)
								for j in C:
									if ([v,i,j,k] in P):
										if (j!=l):
											exprr.add(decvary[v][i][j][k],M)
								for q in C:
									if (q != i):
										if (q != k):
											if (q != l):
												for n in N_plus:
													if ([v,l,q,n] in P):
														if (n != i):
															if (n != k):
																exprr.add(decvary[v][l][q][n],M)
								# Constraint (15): No overlapping sorties
								m.addConstr(exprl, GRB.GREATER_EQUAL, exprr, "Constr.15.%d.%d.%d.%d" % (v,i,k,l))

					# Strengthening Constraint: (NOT IN THE IP MODEL)	
					m.addConstr(decvarchecktprime[v][k] - sR[v][k] - decvarhattprime[v][i] <= quicksum((eee[v][i][j][k] - M)*decvary[v][i][j][k] for j in C if [v,i,j,k] in P) + M, "Constr.xxxxxx.%d.%d.%d" % (v,i,k))

			# Constraint (16): No UAV launch before arrival
			m.addConstr(decvarhattprime[v][i] >= decvarchecktprime[v][i] + sL[v][i] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i)), "Constr.16.%d.%d" % (v,i))	
			
			if (REQUIRE_DRIVER):
				# Constraint (17): No UAV launch before truck arrives if delivery happens after
				m.addConstr(decvarhattprime[v][i] >= decvarcheckt[i] + sL[v][i] - M*(1 - decvarzl[v][0][i]), "Constr.17.%d.%d" % (v,i))

				# Constraint (18): No UAV launch before delivery happens, if it happens before launch	
				m.addConstr(decvarhattprime[v][i] >= decvarbart[i] + sL[v][i] - M*(1 - decvarzl[0][v][i]), "Constr.18.%d.%d" % (v,i))			
			else:
				# Constraint (58): 
				m.addConstr(decvarhattprime[v][i] >= decvarcheckt[i] + sL[v][i] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i)), "Constr.58.%d.%d" % (v,i))	
	
			# Strengthening Constraint: (NOT IN THE IP MODEL)	
			if (REQUIRE_TRUCK_AT_DEPOT):
				m.addConstr(decvarhatt[c+1] >= decvarhattprime[v][i] + quicksum(quicksum( (tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] + sR[v][k]) * decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i), "Constr.x2.%d.%d" % (v,i))


		for k in N_plus:
			# Constraint (7): UAV may rendezvous at any node, including depot, at most once 
			m.addConstr(quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k) <= 1), "Constr.7.%d.%d" % (v, k)

			if (REQUIRE_DRIVER):
				if (REQUIRE_TRUCK_AT_DEPOT):
					# Constraint (23):
					m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] - M*(1 - decvarzr[v][0][k]), "Constr.23.%d.%d" % (v,k))
					# Constraint (24):			
					m.addConstr(decvarchecktprime[v][k] >= decvarbart[k] + sR[v][k] - M*(1 - decvarzr[0][v][k]), "Constr.24.%d.%d" % (v,k))
				else:
					if k != c+1:
						# Constraint (54):
						m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] - M*(1 - decvarzr[v][0][k]), "Constr.54.%d.%d" % (v,k))
						# Constraint (55):
						m.addConstr(decvarchecktprime[v][k] >= decvarbart[k] + sR[v][k] - M*(1 - decvarzr[0][v][k]), "Constr.55.%d.%d" % (v,k))

			else:
				if (REQUIRE_TRUCK_AT_DEPOT):
					# Constraint (59):
					m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for i in N_zero if [v,i,j,k] in P)  for j in C if j != i)) , "Constr.59.%d.%d" % (v,k))			
				else:
					if k != c+1:
						# Constraint (59):
						m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for i in N_zero if [v,i,j,k] in P)  for j in C if j != i)) , "Constr.59.%d.%d" % (v,k))
			
			for j in C:
				if (j != k):
					# Constraint (27):
					m.addConstr(decvarchecktprime[v][k] >= decvarhattprime[v][j] + tauprime[v][j][k] + sR[v][k] - M*(1 - quicksum(decvary[v][i][j][k] for i in N_zero if [v,i,j,k] in P)), "Constr.27.%d.%d.%d" % (v,k,j))
	

			if (REQUIRE_TRUCK_AT_DEPOT):
				# Constraint (34):
				m.addConstr(decvarhatt[k] >= decvarchecktprime[v][k] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k)), "Constr.34.%d.%d" % (v,k))	
			else:
			 	if k != c+1:
			 		# Constraint (34): (Only do this if concerned with minimizing the TRUCK return time, and not the last vehicle)
			 		m.addConstr(decvarhatt[k] >= decvarchecktprime[v][k] - M*(1 - quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k)), "Constr.34.%d.%d" % (v,k))


			# Strengthening Constraint: (NOT IN THE IP MODEL)	
			m.addConstr(quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k) <= quicksum(decvarx[h][k] for h in N_zero if h!=k), "Constr.XXXX.%d.%d" % (v,k))


		for i in C:
			for j in C:
				if (j != i):
					for k in N_plus:
						if ([v,i,j,k] in P):
							# Constraint (8): UAV can only be released from customer node if truck has visited customer node
							m.addConstr(2*decvary[v][i][j][k] <= quicksum(decvarx[h][i] for h in N_zero if h!=i) + quicksum(decvarx[l][k] for l in C if l!=k), "Constr.8.%d.%d.%d.%d" % (v,i,j,k))

							# Constraint (28): Endurance limitations
							m.addConstr(decvarchecktprime[v][k] - sR[v][k] - decvarhattprime[v][i] <= eee[v][i][j][k] + M*(1 - decvary[v][i][j][k]), "Constr.28.%d.%d.%d.%d" % (v,i,j,k))

			
			for k in N_plus:
				if (k != i):
					# Constraint (10): If UAV launches from node i and rendezvous at node k then truck must visit node i then k
					m.addConstr(decvaru[k] - decvaru[i] >= 1 - (c+2)*(1-quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P)), "Constr.10.%d.%d.%d" % (v,i,k))


		for j in C:
			for k in N_plus:
				if ([v,0,j,k] in P):
					# Constraint (9): UAV may depart from depot and return to k only if truck visits k
					m.addConstr(decvary[v][0][j][k] <= quicksum(decvarx[h][k] for h in N_zero if h!=k), "Constr.9.%d.%d.%d" % (v,j,k))


			for i in N_zero:
				if (i != j):
					# Constraint (21):
					m.addConstr(decvarchecktprime[v][j] >= decvarhattprime[v][i] + tauprime[v][i][j] - M*(1 - quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P)), "Constr.21.%d.%d.%d" % (v,j,i))	

					# Constraint (21b):
					m.addConstr(decvarchecktprime[v][j] <= decvarhattprime[v][i] + tauprime[v][i][j] + M*(1 - quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P)), "Constr.21b.%d.%d.%d" % (v,j,i))	
			
			# Constraint (22):
			m.addConstr(decvarhattprime[v][j] >= decvarchecktprime[v][j] + sigmaprime[j]*(quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for i in N_zero if i != j)), "Constr.22.%d.%d" % (v,j))	

			# Constraint (22b):
			m.addConstr(decvarhattprime[v][j] <= decvarchecktprime[v][j] + sigmaprime[j] + M*(1 - quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for i in N_zero if i != j)), "Constr.22b.%d.%d" % (v,j))	

		for k in N_zero:
			if (REQUIRE_TRUCK_AT_DEPOT):
				# Constraint (35):
				m.addConstr(decvarhatt[k] >= decvarhattprime[v][k] - M*(1 - quicksum(quicksum(decvary[v][k][l][q] for q in N_plus if [v,k,l,q] in P) for l in C if l != k)), "Constr.35.%d.%d" % (v, k))
			else:
				if k != 0:
					# Constraint (57):
					m.addConstr(decvarhatt[k] >= decvarhattprime[v][k] - M*(1 - quicksum(quicksum(decvary[v][k][l][q] for q in N_plus if [v,k,l,q] in P) for l in C if l != k)), "Constr.57.%d.%d" % (v, k))
			

		for v2 in V:
			if (v2 != v):
				for k in N_plus:
					# Constraint (25):
					m.addConstr(decvarchecktprime[v][k] >= decvarchecktprime[v2][k] + sR[v][k] - M*(1 - decvarzr[v2][v][k]), "Constr.25.%d.%d.%d" % (v,v2,k))

					# Constraint (37):
					m.addConstr(decvarzr[v][v2][k] <= quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k), "Constr.37.%d.%d.%d" % (v,v2,k))

					# Constraint (38):	
					m.addConstr(decvarzr[v][v2][k] <= quicksum(quicksum(decvary[v2][i][j][k] for j in C if [v2,i,j,k] in P) for i in N_zero if i != k), "Constr.38.%d.%d.%d" % (v,v2,k))

					# Constraint (39):	
					m.addConstr(decvarzr[v][v2][k] + decvarzr[v2][v][k] <= 1, "Constr.39.%d.%d.%d" % (v,v2,k))	

					# Constraint (40):
					m.addConstr(decvarzr[v][v2][k] + decvarzr[v2][v][k] + 1 >= quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k) + quicksum(quicksum(decvary[v2][i][j][k] for j in C if [v2,i,j,k] in P) for i in N_zero if i != k), "Constr.40.%d.%d.%d" % (v,v2,k))	

				for k in C:
					# Constraint (26):
					m.addConstr(decvarchecktprime[v][k] >= decvarhattprime[v2][k] + sR[v][k] - M*(1 - decvarzprime[v2][v][k]), "Constr.26.%d.%d.%d" % (v,v2,k))	

					# Constraint (46):
					m.addConstr(decvarzprime[v2][v][k] <= quicksum(quicksum(decvary[v2][k][l][q] for q in N_plus if [v2,k,l,q] in P) for l in C if l != k), "Constr.46.%d.%d.%d" % (v,v2,k))	

					# Constraint (47):
					m.addConstr(decvarzdp[v2][v][k] <= quicksum(quicksum(decvary[v][k][l][q] for q in N_plus if [v,k,l,q] in P) for l in C if l != k), "Constr.47.%d.%d.%d" % (v,v2,k))

					# Constraint (48):	
					m.addConstr(decvarzprime[v2][v][k] <= quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k), "Constr.48.%d.%d.%d" % (v,v2,k))

					# Constraint (49):	
					m.addConstr(decvarzdp[v2][v][k] <= quicksum(quicksum(decvary[v2][i][j][k] for j in C if [v2,i,j,k] in P) for i in N_zero if i != k), "Constr.49.%d.%d.%d" % (v,v2,k))

					# Constraint (50):
					m.addConstr(decvarzprime[v2][v][k] + decvarzdp[v][v2][k] + 1 >= quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k) + quicksum(quicksum(decvary[v2][k][l][q] for q in N_plus if [v2,k,l,q] in P) for l in C if l != k), "Constr.50.%d.%d.%d" % (v,v2,k))	
					
					# Constraint (51):
					m.addConstr(decvarzprime[v2][v][k] + decvarzdp[v][v2][k] <= 1, "Constr.51.%d.%d.%d" % (v,v2,k))

					# Constraint (52):	
					m.addConstr(decvarzprime[v2][v][k] + decvarzprime[v][v2][k] <= 1, "Constr.52.%d.%d.%d" % (v,v2,k))

					# Constraint (53):	
					m.addConstr(decvarzdp[v2][v][k] + decvarzdp[v][v2][k] <= 1, "Constr.53.%d.%d.%d" % (v,v2,k))	


				for i in N_zero:
					# Constraint (19):
					m.addConstr(decvarhattprime[v][i] >= decvarhattprime[v2][i] + sL[v][i] - M*(1 - decvarzl[v2][v][i]), "Constr.19.%d.%d.%d" % (v,v2,i))	

					# Constraint (42):
					m.addConstr(decvarzl[v][v2][i] <= quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i), "Constr.42.%d.%d.%d" % (v,v2,i))	

					# Constraint (43):
					m.addConstr(decvarzl[v][v2][i] <= quicksum(quicksum(decvary[v2][i][j][k] for k in N_plus if [v2,i,j,k] in P) for j in C if j != i), "Constr.43.%d.%d.%d" % (v,v2,i))	

					# Constraint (44):
					m.addConstr(decvarzl[v][v2][i] + decvarzl[v2][v][i] <= 1, "Constr.44.%d.%d.%d" % (v,v2,i))	

					# Constraint (45):
					m.addConstr(decvarzl[v][v2][i] + decvarzl[v2][v][i] + 1 >= quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i) + quicksum(quicksum(decvary[v2][i][j][k] for k in N_plus if [v2,i,j,k] in P) for j in C if j != i), "Constr.45.%d.%d.%d" % (v,v2,i))	

				for i in C:
					# Constraint (20):
					m.addConstr(decvarhattprime[v2][i] >= decvarchecktprime[v][i] + sL[v2][i] - M*(1 - decvarzdp[v][v2][i]), "Constr.20.%d.%d.%d" % (v,v2,i))	


	for i in C:
		for j in N_plus:
			if (j!=i):
				exprl = LinExpr()
				exprl.clear()
				exprl.add(decvaru[i], 1.0)
				exprl.add(decvaru[j], -1.0)
				exprl.add(decvarx[i][j], c+2)
				exprr = LinExpr()
				exprr.clear()
				exprr.addConstant(c+1)
				# Constraint (11): Subtour elimination for truck
				m.addConstr(exprl, GRB.LESS_EQUAL, exprr, "Constr.11.%d.%d" % (i, j))


		# Determining values for p[i][j], where p[i][j] = 1 if customer i is visited before
		# customer j and p[i][j] = 0 otherwise
		for j in C:
			if (i!=j):	
				exprl = LinExpr()
				exprl.clear()
				exprr = LinExpr()
				exprr.clear()
				
				exprl.add(decvaru[i],1)
				exprl.add(decvaru[j],-1)
				exprl.add(decvarp[i][j],c+2)
				exprr.addConstant(1)
				# Constraint (12):
				m.addConstr(exprl, GRB.GREATER_EQUAL, exprr, "Constr.12.%d.%d" % (i,j))

				exprl2 = LinExpr()
				exprl2.clear()
				exprr2 = LinExpr()
				exprr2.clear()
				
				exprl2.add(decvaru[i],1)
				exprl2.add(decvaru[j],-1)
				exprl2.add(decvarp[i][j],c+2)
				exprr2.addConstant(c+1)
				# Constraint (13):
				m.addConstr(exprl2, GRB.LESS_EQUAL, exprr2, "Constr.13.%d.%d" % (i,j))

				# Constraint (14):
				m.addConstr(decvarp[i][j] + decvarp[j][i] == 1, "Constr.14.%d.%d" % (i,j))
	

	for i in N_zero:
		for j in N_plus:
			if (j != i):
				# Constraint (29): Setting the truck's arrival time.
				m.addConstr(decvarcheckt[j] >= decvarhatt[i] + tau[i][j] - M*(1 - decvarx[i][j]), "Constr.29.%d.%d" % (i,j))	

		# Strengthening Constraint: (NOT IN THE IP MODEL)	
		m.addConstr(decvarhatt[c+1] >= decvarhatt[i] + quicksum(tau[i][j] * decvarx[i][j] for j in N_plus if j != i), "Constr.x1.%d" % (i))


	for k in N_plus:
		# Constraint (30):
		m.addConstr(decvarbart[k] >= decvarcheckt[k] + sigma[k]*(quicksum(decvarx[j][k] for j in N_zero if j != k)), "Constr.30.%d" % (k))

		# Constraint (33):
		m.addConstr(decvarhatt[k] >= decvarbart[k], "Constr.33.%d" % (k))	
	

	if (REQUIRE_DRIVER):
		for k in N_plus:
			for v in V:
				if (REQUIRE_TRUCK_AT_DEPOT):
					# Constraint (31):
					m.addConstr(decvarbart[k] >= decvarchecktprime[v][k] + sigma[k] - M*(1 - decvarzr[v][0][k]), "Constr.31.%d.%d" % (k, v))
				else:
					if k != c+1:
						# Constraint (56):
						m.addConstr(decvarbart[k] >= decvarchecktprime[v][k] + sigma[k] - M*(1 - decvarzr[v][0][k]), "Constr.56.%d.%d" % (k, v))			


		for k in C:
			for v in V:
				# Constraint (32):
				m.addConstr(decvarbart[k] >= decvarhattprime[v][k] + sigma[k] - M*(1 - decvarzl[v][0][k]), "Constr.32.%d.%d" % (k, v))			


		for v in V:
			for k in N_plus:
				# Constraint (36):
				m.addConstr(decvarzr[0][v][k] + decvarzr[v][0][k] == quicksum(quicksum(decvary[v][i][j][k] for j in C if [v,i,j,k] in P) for i in N_zero if i != k), "Constr.36.%d.%d" % (v,k))	

	
			for i in N_zero:
				# Constraint (41):
				m.addConstr(decvarzl[0][v][i] + decvarzl[v][0][i] == quicksum(quicksum(decvary[v][i][j][k] for k in N_plus if [v,i,j,k] in P) for j in C if j != i), "Constr.41.%d.%d" % (v,i))	

	
	# Solve
	m.optimize()

	# Check feasibility of the model:
	if (m.Status == GRB.INFEASIBLE):
		# NO FEASIBLE SOLUTION EXISTS
		
		OFV 			= -2	# Infeasible
		assignments		= []
		packages		= []
		isOptimal 		= False
		bestBound		= -2
		maxActualSkips	= -2
		waitingTruck 	= -1
		waitingUAV		= -1

	
	elif ((m.Status == GRB.TIME_LIMIT) and (m.objVal > 1e30)):
		# NO FEASIBLE SOLUTION WAS FOUND (maybe one exists, but we ran out of time)
		
		OFV 			= -1	# Couldn't find an incumbent
		assignments		= []
		packages		= []
		isOptimal 		= False
		bestBound		= -1
		maxActualSkips	= -1
		waitingTruck 	= -1
		waitingUAV		= -1
	
	else:
		# We found a feasible solution
		print('\nOBJECTIVE FUNCTION VALUE: %f' % (m.objVal))

		OFV = m.objVal
	
		bestBound = m.ObjBound
		if (m.Status == GRB.TIME_LIMIT):
			isOptimal = False
		else:
			isOptimal = True
		
		waitingTruck 	= 0.0
		waitingUAV		= 0.0
	
		# BUILD ASSIGNMENTS AND PACKAGES DICTIONARIES:

		packages = {}	# Datastructure to keep track of delivery type and delivery time of packages
		assignments = defaultdict(make_dict)	# Datastructure to keep track of tasks of different vehicles
	
	
		# --------------------------------
		x = []
		i = 0
		# Truck only!
		for k in N_zero:
			for j in N_plus:
				if (i != j) and (i != c+1):
					if (decvarx[i][j].x > 0.2):
						x.append([i,j])
						packages[j] = make_packages(TYPE_TRUCK, node[j].latDeg, node[j].lonDeg, decvarbart[j].x, packageIcons[1])
	
						print('x[%d][%d] = %f' % (i,j,decvarx[i][j].x))
						print('\t Depart from %d at %f' % (i, decvarhatt[i].x))
						print('\t tau[%d][%d] = %f' % (i, j, tau[i][j]))
						print('\t Arrive to %d at %f' % (j, decvarcheckt[j].x))
						print('\t sigma[%d] = %f' % (j, sigma[j]))
						print('\t Complete Service: %f' % (decvarbart[j].x))
						print('\t Depart from %d at %f' % (j, decvarhatt[j].x))
						
						i = j
	
						break
	
		y = []				
		for v in V:
			# UAVs only!
			for i in N_zero:
				for j in C:
					if (j != i):
						for k in N_plus:
							if ([v,i,j,k] in P):
								if (decvary[v][i][j][k].x > 0.2):
									y.append([v,i,j,k])
									packages[j] = make_packages(TYPE_UAV, node[j].latDeg, node[j].lonDeg, decvarhattprime[v][j].x, packageIcons[0])
	
									print('y[%d][%d][%d][%d] = %f' % (v,i,j,k,decvary[v][i][j][k].x))
									print('\t Arrive at i = %d: %f' % (i, decvarchecktprime[v][i].x))
									print('\t Launch from i = %d: %f' % (i, decvarhattprime[v][i].x))
									print('\t tauprime[%d][%d][%d] = %f' % (v, i, j, tauprime[v][i][j]))
									print('\t Arrive at cust j = %d: %f' % (j, decvarchecktprime[v][j].x))
									print('\t sigmaprime[%d] = %f' % (j, sigmaprime[j]))
									print('\t Depart cust j = %d: %f' % (j, decvarhattprime[v][j].x))
									print('\t tauprime[%d][%d][%d] = %f' % (v,j,k,tauprime[v][j][k]))
									print('\t Arrive at k = %d: %f' % (k, decvarchecktprime[v][k].x))
									print('\t Depart from k = %d: %f' % (k, decvarhattprime[v][k].x))
									
	
		# Capture all UAVs that land at a particular node
		# Capture all UAVs that launch from a particular node
		launchesfrom = {}
		landsat = {}
		for i in N:
			launchesfrom[i] = []
			landsat[i] = []
		for [v,i,j,k] in y:
			launchesfrom[i].append(v)
			landsat[k].append(v)
	
	
		# Build the truck's route in order
		prevTime = {}
		assignmentsArray = {}
		packagesArray = {}
		
		prevTime[1] = 0.0	# truck
		assignmentsArray[1] = []
		for v in V:
			prevTime[v] = 0.0	# UAVs
			assignmentsArray[v] = []
				
		# Are there any UAVs on the truck?
		uavRiders = []
		for v in V:
			uavRiders.append(v)
	
		tmpIcon = 'ub_truck_%d.gltf' % (1)	
		for [i,j] in x:
			# Capture the waiting time
			waitingTruck += ((decvarcheckt[j].x - decvarcheckt[i].x) - (tau[i][j] + sigma[j]))
			

			# Are there any UAVs on the truck when the truck leaves i?
			for v in V:
				if ((v in landsat[i]) and (v not in uavRiders)):
					uavRiders.append(v)
				if ((v in launchesfrom[i]) and (v in uavRiders)):
					uavRiders.remove(v)
	
	
			# These activities need to be sorted by time (ascending)
			tmpTimes = []
			
			if (i == 0 and REQUIRE_TRUCK_AT_DEPOT):
				for v in launchesfrom[i]:
					if (len(uavRiders) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = decvarhattprime[v][i].x - sL[v][i]
					A_startNodeID = i
					A_startLatDeg = node[i].latDeg
					A_startLonDeg = node[i].lonDeg
					A_startAltMeters = 0.0
					A_endTime = decvarhattprime[v][i].x
					A_endNodeID = i
					A_endLatDeg = node[i].latDeg
					A_endLonDeg = node[i].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Launching UAV %d' % (v)
					A_UAVsOnBoard = uavRiders
					A_ganttStatus = GANTT_LAUNCH
			
					tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
		

			if (len(uavRiders) > 0):
				A_statusID = TRAVEL_TRUCK_W_UAV
			else:
				A_statusID = TRAVEL_TRUCK_EMPTY
			A_vehicleType = TYPE_TRUCK
			A_startTime = decvarhatt[i].x
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = decvarhatt[i].x + tau[i][j]
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = tmpIcon
			A_description = 'Travel from node %d to node %d' % (i, j)
			A_UAVsOnBoard = uavRiders
			A_ganttStatus = GANTT_TRAVEL
	
	
			tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
	
			if (decvarcheckt[j].x - decvarhatt[i].x - tau[i][j] > 0.01):
				if (len(uavRiders) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = (decvarhatt[i].x + tau[i][j])
				A_startNodeID = j
				A_startLatDeg = node[j].latDeg
				A_startLonDeg = node[j].lonDeg
				A_startAltMeters = 0.0
				A_endTime = decvarcheckt[j].x
				A_endNodeID = j
				A_endLatDeg = node[j].latDeg
				A_endLonDeg = node[j].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Idle for %3.0f seconds at node %d' % (A_endTime - A_startTime, j)
				A_UAVsOnBoard = uavRiders
				A_ganttStatus = GANTT_IDLE
	
				tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		

			if (j == c+1):
				myMin, mySec	= divmod(decvarhatt[j].x, 60)
				myHour, myMin 	= divmod(myMin, 60)
				A_description	= 'At the Depot.  Total Time = %d:%02d:%02d' % (myHour, myMin, mySec) 
				A_endTime		= -1
				A_ganttStatus	= GANTT_FINISHED
			else:
				A_description		= 'Dropping off package to Customer %d' % (j)
				A_endTime 		= decvarbart[j].x
				A_ganttStatus 	= GANTT_DELIVER
				packagesArray[j] = [TYPE_TRUCK, node[j].latDeg, node[j].lonDeg, decvarbart[j].x, packageIcons[1]]

		
			if (len(uavRiders) > 0):
				A_statusID = STATIONARY_TRUCK_W_UAV
			else:
				A_statusID = STATIONARY_TRUCK_EMPTY
			A_vehicleType = TYPE_TRUCK
			if (j == c+1):
				A_startTime = decvarhatt[j].x - sigma[j]
			else:	
				A_startTime = decvarbart[j].x - sigma[j]
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = tmpIcon
			A_UAVsOnBoard = uavRiders
	
			tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
			if (REQUIRE_TRUCK_AT_DEPOT and j <= c+1) or (not REQUIRE_TRUCK_AT_DEPOT and j < c+1):
				# We're NOT going to ignore UAVs that land at the depot.
				for v in landsat[j]:
					if (len(uavRiders) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = decvarchecktprime[v][j].x - sR[v][j]
					A_startNodeID = j
					A_startLatDeg = node[j].latDeg
					A_startLonDeg = node[j].lonDeg
					A_startAltMeters = 0.0
					A_endTime = decvarchecktprime[v][j].x
					A_endNodeID = j
					A_endLatDeg = node[j].latDeg
					A_endLonDeg = node[j].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Retrieving UAV %d' % (v)
					A_UAVsOnBoard = uavRiders
					A_ganttStatus = GANTT_RECOVER
			
					tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
	
	
			for v in launchesfrom[j]:
				if (len(uavRiders) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = decvarhattprime[v][j].x - sL[v][j]
				A_startNodeID = j
				A_startLatDeg = node[j].latDeg
				A_startLonDeg = node[j].lonDeg
				A_startAltMeters = 0.0
				A_endTime = decvarhattprime[v][j].x
				A_endNodeID = j
				A_endLatDeg = node[j].latDeg
				A_endLonDeg = node[j].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Launching UAV %d' % (v)
				A_UAVsOnBoard = uavRiders
				A_ganttStatus = GANTT_LAUNCH
		
				tmpTimes.append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
	
			# Now, sort the tmpTimes array based on ascending start times.  
			# Along the way, check for truck idle times.
			unasgnInd = range(0, len(tmpTimes))		
			while (len(unasgnInd) > 0):
				tmpMin = 2*decvarhatt[j].x	# Set to a large number
				# Find the minimum unassigned time
				for tmpIndex in unasgnInd:
					if (tmpTimes[tmpIndex][2] < tmpMin):
						tmpMin = tmpTimes[tmpIndex][2]	# This is the "startTime" component of tmpTimes
						myIndex = tmpIndex
						
				# Was there idle time in the assignments?
				if (tmpMin - prevTime[1] > 0.01):
					# MAKE ASSIGNMENT:	
					if (len(tmpTimes[myIndex][14]) > 0):
						A_statusID = STATIONARY_TRUCK_W_UAV
					else:
						A_statusID = STATIONARY_TRUCK_EMPTY
					A_vehicleType = TYPE_TRUCK
					A_startTime = prevTime[1]
					A_startNodeID = tmpTimes[myIndex][3]
					A_startLatDeg = node[tmpTimes[myIndex][3]].latDeg
					A_startLonDeg = node[tmpTimes[myIndex][3]].lonDeg
					A_startAltMeters = 0.0
					A_endTime = prevTime[1] + (tmpMin - prevTime[1])
					A_endNodeID = tmpTimes[myIndex][3]
					A_endLatDeg = node[tmpTimes[myIndex][3]].latDeg
					A_endLonDeg = node[tmpTimes[myIndex][3]].lonDeg
					A_endAltMeters = 0.0
					A_icon = tmpIcon
					A_description = 'Idle for %3.0f seconds' % (A_endTime - A_startTime)
					A_UAVsOnBoard = tmpTimes[myIndex][14]
					A_ganttStatus = GANTT_IDLE
			
					assignmentsArray[1].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		

				# MAKE ASSIGNMENT:
				assignmentsArray[1].append(tmpTimes[myIndex])
							
				prevTime[1] = tmpTimes[myIndex][7] 		# This is the "endTime" component of tmpTimes
				unasgnInd.remove(myIndex)
				
			# Also, is there idle time before leaving node j?  Check prevTime[1] and decvarhatt[j].x 
			if ((j != c+1) and (prevTime[1] - decvarhatt[j].x < -0.01)):
				# MAKE ASSIGNMENT:			
				if (len(tmpTimes[myIndex][14]) > 0):
					A_statusID = STATIONARY_TRUCK_W_UAV
				else:
					A_statusID = STATIONARY_TRUCK_EMPTY
				A_vehicleType = TYPE_TRUCK
				A_startTime = tmpMin
				A_startNodeID = tmpTimes[myIndex][3]
				A_startLatDeg = node[tmpTimes[myIndex][3]].latDeg
				A_startLonDeg = node[tmpTimes[myIndex][3]].lonDeg
				A_startAltMeters = 0.0
				A_endTime = decvarhatt[j].x
				A_endNodeID = tmpTimes[myIndex][3]
				A_endLatDeg = node[tmpTimes[myIndex][3]].latDeg
				A_endLonDeg = node[tmpTimes[myIndex][3]].lonDeg
				A_endAltMeters = 0.0
				A_icon = tmpIcon
				A_description = 'Idle for %3.0f seconds before departing' % (A_endTime - A_startTime)
				A_UAVsOnBoard = tmpTimes[myIndex][14]
				A_ganttStatus = GANTT_IDLE
		
				assignmentsArray[1].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])					
	
			# Update the previous time value:					
			prevTime[1] = decvarhatt[j].x
	

		for [v,i,j,k] in y:
			# Capture waiting time for UAVs
			waitingUAV += ((decvarchecktprime[v][k].x - decvarchecktprime[v][i].x) - (tauprime[v][i][j] + tauprime[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))
			
			# Launch Prep (on ground, with package)		
			A_statusID = STATIONARY_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = decvarhattprime[v][i].x - sL[v][i]
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = decvarhattprime[v][i].x
			A_endNodeID = i
			A_endLatDeg = node[i].latDeg
			A_endLonDeg = node[i].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Prepare to launch from truck'
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_LAUNCH
	
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
	
			# Takoff (vertical, with package)
			A_statusID = VERTICAL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = decvarhattprime[v][i].x
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = 0.0
			A_endTime = decvarhattprime[v][i].x + travel[v][i][j].takeoffTime
			A_endNodeID = i
			A_endLatDeg = node[i].latDeg
			A_endLonDeg = node[i].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			if (i == 0):
				A_description = 'Takeoff from Depot'
			else:	
				A_description = 'Takeoff from truck at Customer %d' % (i)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
	
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
			
			tmpStart = decvarhattprime[v][i].x + travel[v][i][j].takeoffTime
			# Idle (i --> j)?
			if (decvarchecktprime[v][j].x - decvarhattprime[v][i].x - travel[v][i][j].totalTime > 0.01):
				tmpIdle = decvarchecktprime[v][j].x - (decvarhattprime[v][i].x + travel[v][i][j].totalTime)
				tmpEnd = tmpStart + tmpIdle
				A_statusID = STATIONARY_UAV_PACKAGE
				A_vehicleType = TYPE_UAV
				A_startTime = tmpStart
				A_startNodeID = i
				A_startLatDeg = node[i].latDeg
				A_startLonDeg = node[i].lonDeg
				A_startAltMeters = vehicle[v].cruiseAlt
				A_endTime = tmpEnd
				A_endNodeID = i
				A_endLatDeg = node[i].latDeg
				A_endLonDeg = node[i].lonDeg
				A_endAltMeters = vehicle[v].cruiseAlt
				A_icon = 'iris_black_blue_plus_box_yellow.gltf'
				A_description = 'Idle at initial takeoff (node %d)' % (i)
				A_UAVsOnBoard = []
				A_ganttStatus = GANTT_IDLE
				
				assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
				tmpStart = tmpEnd
	
				
			# Fly to customer j (with package)
			A_statusID = TRAVEL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = tmpStart
			A_startNodeID = i
			A_startLatDeg = node[i].latDeg
			A_startLonDeg = node[i].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			A_endTime = tmpStart + travel[v][i][j].flyTime
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Fly to UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
			
			
			# Land at customer (Vertical, with package)
			A_statusID = VERTICAL_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = decvarchecktprime[v][j].x - travel[v][i][j].landTime
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			A_endTime = decvarchecktprime[v][j].x
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Land at UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
			
	
			# Serve customer (on ground, with package)
			A_statusID = STATIONARY_UAV_PACKAGE
			A_vehicleType = TYPE_UAV
			A_startTime = decvarchecktprime[v][j].x
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			A_endTime = decvarhattprime[v][j].x
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_black_blue_plus_box_yellow.gltf'
			A_description = 'Serving UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_DELIVER
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
			packagesArray[j] = [TYPE_UAV, node[j].latDeg, node[j].lonDeg, decvarhattprime[v][j].x, packageIcons[0]]
	
			
			# Takeoff (vertical, empty)
			A_statusID = VERTICAL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = decvarhattprime[v][j].x
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = 0.0
			if (k == c+1):
				# We didn't define "travel" ending at the depot replica.
				A_endTime = decvarhattprime[v][j].x + travel[v][j][0].takeoffTime
			else:
				A_endTime = decvarhattprime[v][j].x + travel[v][j][k].takeoffTime
			A_endNodeID = j
			A_endLatDeg = node[j].latDeg
			A_endLonDeg = node[j].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_with_props_black_blue.gltf'
			A_description = 'Takeoff from UAV customer %d' % (j)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
			
			# Fly to truck (empty)
			A_statusID = TRAVEL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			if (k == c+1):
				# We didn't define "travel" ending at the depot replica.
				A_startTime = decvarhattprime[v][j].x + travel[v][j][0].takeoffTime
			else:
				A_startTime = decvarhattprime[v][j].x + travel[v][j][k].takeoffTime
			A_startNodeID = j
			A_startLatDeg = node[j].latDeg
			A_startLonDeg = node[j].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			if (k == c+1):
				# We didn't define "travel" ending at the depot replica.
				A_endTime = decvarhattprime[v][j].x + travel[v][j][0].takeoffTime + travel[v][j][0].flyTime
			else:
				A_endTime = decvarhattprime[v][j].x + travel[v][j][k].takeoffTime + travel[v][j][k].flyTime
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = vehicle[v].cruiseAlt
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Fly to depot'
			else:
				A_description = 'Fly to truck at customer %d' % (k)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
			
			# Idle (j --> k)?
			if (k == c+1):
				# We didn't define "travel" ending at the depot replica.
				tmpStart =  decvarhattprime[v][j].x + travel[v][j][0].takeoffTime + travel[v][j][0].flyTime
				tmpIdle = decvarchecktprime[v][k].x - sR[v][k] - decvarhattprime[v][j].x - travel[v][j][0].totalTime
			else:
				tmpStart =  decvarhattprime[v][j].x + travel[v][j][k].takeoffTime + travel[v][j][k].flyTime
				tmpIdle = decvarchecktprime[v][k].x - sR[v][k] - decvarhattprime[v][j].x - travel[v][j][k].totalTime
				
			if (tmpIdle > 0.01):				
				tmpEnd = tmpStart + tmpIdle
				A_statusID = STATIONARY_UAV_EMPTY
				A_vehicleType = TYPE_UAV
				A_startTime = tmpStart
				A_startNodeID = k
				A_startLatDeg = node[k].latDeg
				A_startLonDeg = node[k].lonDeg
				A_startAltMeters = vehicle[v].cruiseAlt
				A_endTime = tmpEnd
				A_endNodeID = k
				A_endLatDeg = node[k].latDeg
				A_endLonDeg = node[k].lonDeg
				A_endAltMeters = vehicle[v].cruiseAlt
				A_icon = 'iris_with_props_black_blue.gltf'
				if (k == c+1):
					A_description = 'Idle above depot location'
				else:
					A_description = 'Idle above rendezvous location (customer %d)' % (k)					
				A_UAVsOnBoard = []
				A_ganttStatus = GANTT_IDLE
				
				assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
	
				tmpStart = tmpEnd
			
			
			# Land at k (vertical, empty)
			A_statusID = VERTICAL_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = tmpStart
			A_startNodeID = k
			A_startLatDeg = node[k].latDeg
			A_startLonDeg = node[k].lonDeg
			A_startAltMeters = vehicle[v].cruiseAlt
			if (k == c+1):
				# We didn't define "travel" ending at the depot replica.
				A_endTime = tmpStart + travel[v][j][0].landTime
			else:
				A_endTime = tmpStart + travel[v][j][k].landTime
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Land at depot'
			else:
				A_description = 'Land at truck rendezvous location (customer %d)' % (k)
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_TRAVEL
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		
			
			
			# Recovery (on ground, empty)
			A_statusID = STATIONARY_UAV_EMPTY
			A_vehicleType = TYPE_UAV
			A_startTime = decvarchecktprime[v][k].x - sR[v][k]
			A_startNodeID = k
			A_startLatDeg = node[k].latDeg
			A_startLonDeg = node[k].lonDeg
			A_startAltMeters = 0.0
			A_endTime = decvarchecktprime[v][k].x
			A_endNodeID = k
			A_endLatDeg = node[k].latDeg
			A_endLonDeg = node[k].lonDeg
			A_endAltMeters = 0.0
			A_icon = 'iris_with_props_black_blue.gltf'
			if (k == c+1):
				A_description = 'Recovered at depot'
			else:
				A_description = 'Recovered by truck at customer %d' % k
			A_UAVsOnBoard = []
			A_ganttStatus = GANTT_RECOVER
			
			assignmentsArray[v].append([A_statusID, A_vehicleType, A_startTime, A_startNodeID, A_startLatDeg, A_startLonDeg, A_startAltMeters, A_endTime, A_endNodeID, A_endLatDeg, A_endLonDeg, A_endAltMeters, A_icon, A_description, A_UAVsOnBoard, A_ganttStatus])		

	
		# Convert assignmentsArray into the assignments class:
		for v in vehicle:
			for i in range(0, len(assignmentsArray[v])):
				statusID = assignmentsArray[v][i][0]
				if (0 in assignments[v][statusID]):
					statusIndex = len(assignments[v][statusID])
				else:
					statusIndex = 0
				
				assignments[v][statusID][statusIndex] = make_assignments(assignmentsArray[v][i][1], assignmentsArray[v][i][2], assignmentsArray[v][i][3], assignmentsArray[v][i][4], assignmentsArray[v][i][5], assignmentsArray[v][i][6], assignmentsArray[v][i][7], assignmentsArray[v][i][8], assignmentsArray[v][i][9], assignmentsArray[v][i][10], assignmentsArray[v][i][11], assignmentsArray[v][i][12], assignmentsArray[v][i][13], assignmentsArray[v][i][14], assignmentsArray[v][i][15])	

		
	return(OFV, assignments, packages, isOptimal, bestBound, waitingTruck, waitingUAV) 