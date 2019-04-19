#!/usr/bin/env python

import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from collections import defaultdict

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

# There's a package color that corresponds to the VEHICLE that delivered the package.
# Right now we only have 5 boxes (so we can have at most 5 trucks).
packageIcons		= ['box_yellow_centered.gltf', 'box_blue_centered.gltf', 'box_orange_centered.gltf', 'box_green_centered.gltf', 'box_gray_centered.gltf', 'box_brown_centered.gltf']
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


def mfstsp_heuristic_3_timing(x, y, z, node, eee, N, P, V, cutoffTime, c, sigma, sigmaprime, tau, tauprime, minDistance, sR, sL, vehicle, travel, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, optLowBnd):
	# Solve P3 here using Gurobi (Line 3 in Algorithm 6)

	# Find the cost of inserting truck customers:
	insertCost = {}
	insertPairs = {}
	for j in node:
		if (node[j].nodeType == NODE_TYPE_CUST):
			insertCost[j] = float('inf')
			for [i,k] in x:
				tmpCost = max(0, (tau[i][j] + tau[j][k] - tau[i][k]))
				if (tmpCost < insertCost[j]):
					insertCost[j] = tmpCost	
					insertPairs[j] = [i, k]	

	# Re-calculate P, based on previous phases
	P = []
	B = defaultdict(make_dict)	# B(v,k) parameter in Table 5
	A = defaultdict(make_dict)	# A(v,i) parameter in Table 5
	for [v,i,j,k] in y:
		P.append([v,i,j,k])
		B[v][k] = j
		A[v][i] = j		

	# Reset N_zero -- Should only include customers visted by the truck, plus depot 0.	
	# Reset N_plus -- Should only include customers visted by the truck, plus depot c+1.	
	N_zero = []
	N_plus = []
	N_zero.append(0)
	for [i,j] in x:
		if (i not in [0, c+1]):
			N_zero.append(i)
			N_plus.append(i)
	
	N_plus.append(c+1)
	
	# Reset C -- Should only include customers NOT visited by the truck
	C = []
	for [v,i,j,k] in y:
		
		if (j not in C):
			C.append(j)

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
		
		
	# 2) GUROBI
	# Model
	m = Model("phase3")

	# Tell Gurobi not to print to a log file
	m.params.OutputFlag = 0


	# a) Decision Variable Definitions and Objective Function:

	decvarcheckt 	= defaultdict(make_dict)
	decvarbart 		= defaultdict(make_dict)
	decvarhatt 		= defaultdict(make_dict)

	decvarchecktprime 	= defaultdict(make_dict)
	decvarhattprime 	= defaultdict(make_dict)

	decvarzl 		= defaultdict(make_dict)
	decvarzr 		= defaultdict(make_dict)
	decvarzprime 	= defaultdict(make_dict)
	decvarzdp 		= defaultdict(make_dict)
	decvarzhat		= {}
	

	RELAX = False
	if (RELAX):
		myVtype = GRB.CONTINUOUS
	else:
		myVtype = GRB.BINARY
					
	
	for i in N:
		if (i in C):
			decvarzhat[i] = m.addVar(lb = 0, ub = 1, obj = optLowBnd + 10*insertCost[i], vtype = GRB.BINARY, name = "hatz.%d" % (i))
			
		if (REQUIRE_TRUCK_AT_DEPOT):
			if (i == c+1):
				# This is our only term in the objective function (for hatt):
				decvarhatt[i] = m.addVar(lb = 0, obj=1, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			elif (i == 0):
				# DON'T Hard-code \hat{t}_{0} = 0.
				# However, \check{t}_{0} = \bar{t}_{0} = 0
				decvarhatt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))			
				decvarbart[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, ub = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			
	
			elif (i not in C):
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
	
			elif (i not in C):
				# Just a regular non-negative continuous decision variable:
				decvarhatt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hatt.%d" % (i))
				decvarbart[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="bart.%d" % (i))			
				decvarcheckt[i] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checkt.%d" % (i))			

	for v in V:
		# Hard-code to zero:
		decvarchecktprime[v][0] = m.addVar(lb = 0, ub=0, obj=0, vtype=GRB.CONTINUOUS, name="checktprime.%d.0" % (v))

	for k in N:
		for v in V:
			if ((v in landsat[k]) or (v in launchesfrom[k])):
				if (k != 0):
					decvarchecktprime[v][k] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checktprime.%d.%d" % (v, k))
				decvarhattprime[v][k] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hattprime.%d.%d" % (v, k))

	for [v,i,j,k] in y:
		decvarchecktprime[v][j] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="checktprime.%d.%d" % (v, j))
		decvarhattprime[v][j] = m.addVar(lb = 0, obj=0, vtype=GRB.CONTINUOUS, name="hattprime.%d.%d" % (v, j))
					

	if (REQUIRE_DRIVER):
		for v in V:
			# Hardcode z^{R}_{v, 0, c+1} = 0 for all v in V:
			decvarzr[v][0][c+1] = m.addVar(lb = 0, ub = 0, obj = 0, vtype=myVtype, name="zr.%d.0.%d" % (v,c+1))
			if (REQUIRE_TRUCK_AT_DEPOT):
				# DON'T Hardcode z^{L}_{v, 0, 0} = 0 for all v in V:
				decvarzl[v][0][0] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zl.%d.0.0" % (v))
			else:
				# Hardcode z^{L}_{v, 0, 0} = 0 for all v in V:
				decvarzl[v][0][0] = m.addVar(lb = 0, ub = 0, obj = 0, vtype=myVtype, name="zl.%d.0.0" % (v))
	
			
	for k in N_plus:
		if (REQUIRE_DRIVER):
			zzz = set().union([0], landsat[k])
		else:
			zzz = landsat[k]
		for v1 in zzz:
			for v2 in zzz:
				if (v1 != v2):
					if ((v2 == 0) and (k == c+1 or k == 0)):
						print "skip"
					else:	
						decvarzr[v1][v2][k] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zr.%d.%d.%d" % (v1,v2,k))

	for i in N_zero:
		if (REQUIRE_DRIVER):
			zzz = set().union([0], launchesfrom[i])
		else:
			zzz = launchesfrom[i]	
		for v1 in zzz:
			for v2 in zzz:
				if (v1 != v2):
					if (v2 == 0 and (i == c+1 or i == 0)):
						print "skip"
					else:
						decvarzl[v1][v2][i] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zl.%d.%d.%d" % (v1,v2,i))
					
	for i in N_plus:
		if (i != 0):
			for v1 in launchesfrom[i]:
				for v2 in landsat[i]:
					# NOTE:  v1 can equal v2, since v can land at and then launch from the same location.
					decvarzprime[v1][v2][i] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zprime.%d.%d.%d" % (v1,v2,i))
			for v1 in landsat[i]:
				for v2 in launchesfrom[i]:
					# NOTE:  v1 can equal v2, since v can land at and then launch from the same location.
					decvarzdp[v1][v2][i] = m.addVar(lb = 0, ub = 1, obj = 0, vtype=myVtype, name="zdp.%d.%d.%d" % (v1,v2,i))
					

	
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
	
	# Set a tolerance for MIP gap
	m.params.MIPgap = 0.03	# 3%
	
	# Update model to integrate new variables
	m.update()

	# Start adding constraints:

	for k in N_plus:
		for v in landsat[k]:
			if (REQUIRE_TRUCK_AT_DEPOT):
				# Constraint (72):
				m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] - M*decvarzhat[B[v][k]], "Constr.72.%d.%d" % (v,k))		
				if (REQUIRE_DRIVER):
					# Constraint (73):	
					m.addConstr(decvarchecktprime[v][k] >= decvarbart[k] + sR[v][k] - M*(1 - decvarzr[0][v][k]), "Constr.73.%d.%d" % (v,k))			

			elif (k != c+1):
				# Constraint (72): We don't require the truck to be at the depot for landing.  Ignore when k == c+1
				m.addConstr(decvarchecktprime[v][k] >= decvarcheckt[k] + sR[v][k] -M*decvarzhat[B[v][k]], "Constr.72.%d.%d" % (v,k))
				if (REQUIRE_DRIVER):
					# Constraint (73):
					m.addConstr(decvarchecktprime[v][k] >= decvarbart[k] + sR[v][k] - M*(1 - decvarzr[0][v][k]), "Constr.73.%d.%d" % (v,k))		

			for v2 in landsat[k]:
				if (v2 != v):
					# Constraint (74):
					m.addConstr(decvarchecktprime[v][k] >= decvarchecktprime[v2][k] + sR[v][k] - M*(1 - decvarzr[v2][v][k]), "Constr.74.%d.%d.%d" % (v,v2,k))
			
			if (k != c+1):
				for v2 in launchesfrom[k]:
					if (v2 != v):
						# Constraint (75):
						m.addConstr(decvarchecktprime[v][k] >= decvarhattprime[v2][k] + sR[v][k] - M*(1 - decvarzprime[v2][v][k]), "Constr.75.%d.%d.%d" % (v,v2,k))	

	for [v,i,j,k] in y:
		# Constraint (76):
		m.addConstr(decvarchecktprime[v][k] >= decvarhattprime[v][j] + (tauprime[v][j][k] + sR[v][k])*(1-decvarzhat[j]), "Constr.76.%d.%d.%d" % (v,k,j))

		# Constraint (70):	
		m.addConstr(decvarchecktprime[v][j] >= decvarhattprime[v][i] + tauprime[v][i][j]*(1-decvarzhat[j]), "Constr.70.%d.%d.%d" % (v,j,i))

		# Constraint (71):
		m.addConstr(decvarhattprime[v][j] >= decvarchecktprime[v][j] + sigmaprime[j]*(1-decvarzhat[j]), "Constr.71.%d.%d" % (v,j))

		# Constraint (77):	
		m.addConstr(decvarchecktprime[v][k] - sR[v][k] - decvarhattprime[v][i] <= eee[v][i][j][k], "Constr.77.%d.%d.%d" % (v,i,k))	


	for i in N_zero:
		for v in launchesfrom[i]:
			# Constraint (65):
			m.addConstr(decvarhattprime[v][i] >= decvarchecktprime[v][i] + sL[v][i]*(1-decvarzhat[A[v][i]]), "Constr.65.%d.%d" % (v,i))	
			
			if (REQUIRE_DRIVER):
				# Constraint (66):
				m.addConstr(decvarhattprime[v][i] >= decvarcheckt[i] + sL[v][i] - M*(1 - decvarzl[v][0][i]), "Constr.66.%d.%d" % (v,i))

				# Constraint (67):
				m.addConstr(decvarhattprime[v][i] >= decvarbart[i] + sL[v][i] - M*(1 - decvarzl[0][v][i]), "Constr.67.%d.%d" % (v,i))			
			
			else:
				# Constraint (104):
				m.addConstr(decvarhattprime[v][i] >= decvarcheckt[i] + sL[v][i] - M*(decvarzhat[A[v][i]]), "Constr.104.%d.%d" % (v,i))
			
			for v2 in launchesfrom[i]:
				if (v2 != v):
					# Constraint (68):
					m.addConstr(decvarhattprime[v][i] >= decvarhattprime[v2][i] + sL[v][i] - M*(1 - decvarzl[v2][v][i]), "Constr.68.%d.%d.%d" % (v,v2,i))			

		if (i != 0):
			for v in landsat[i]:
				for v2 in launchesfrom[i]:
					if (v2 != v):
						# Constraint (69):
						m.addConstr(decvarhattprime[v2][i] >= decvarchecktprime[v][i] + sL[v2][i] - M*(1 - decvarzdp[v][v2][i]), "Constr.69.%d.%d.%d" % (v,v2,i))	


	for [i,j] in x:
		# Constraint (78):
		m.addConstr(decvarcheckt[j] >= decvarhatt[i] + tau[i][j], "Constr.78.%d.%d" % (i,j))	


	for k in N_plus:
		# Constraint (83):
		m.addConstr(decvarhatt[k] >= decvarbart[k], "Constr.83.%d" % (k))			

		if (k != c+1):
			# Constraint (79):
			m.addConstr(decvarbart[k] >= decvarcheckt[k] + sigma[k], "Constr.79.%d" % (k))	

			for v in launchesfrom[k]:
				# Constraint (85):
				m.addConstr(decvarhatt[k] >= decvarhattprime[v][k], "Constr.85.%d.%d" % (v, k))

				if (REQUIRE_DRIVER):
					# Constraint (82):
					m.addConstr(decvarbart[k] >= decvarhattprime[v][k] + sigma[k] - M*(1 - decvarzl[v][0][k]), "Constr.82.%d" % (k))			

		else:
			# Constraint (80):
			m.addConstr(decvarbart[k] >= decvarcheckt[k], "Constr.80.%d" % (k))	
	
		for v in landsat[k]:
			# Constraint (84):
			m.addConstr(decvarhatt[k] >= decvarchecktprime[v][k], "Constr.84.%d.%d" % (v,k))

			if (REQUIRE_DRIVER):
				# Constraint (81):
				m.addConstr(decvarbart[k] >= decvarchecktprime[v][k] + sigma[k] - M*(1 - decvarzr[v][0][k]), "Constr.81.%d" % (k))

				# Constraint (87):
				m.addConstr(decvarzr[0][v][k] + decvarzr[v][0][k] + decvarzhat[B[v][k]] == 1, "Constr.87.%d.%d" % (v,k))

			for v2 in landsat[k]:
				if (v2 != v):
					# Constraint (88):
					m.addConstr(decvarzr[v][v2][k] + decvarzr[v2][v][k] <= 1, "Constr.88.%d.%d.%d" % (v,v2,k))

					# Constraint (89):	
					m.addConstr(decvarzr[v][v2][k] + decvarzr[v2][v][k] + decvarzhat[B[v][k]] + decvarzhat[B[v2][k]] >= 1, "Constr.89.%d.%d.%d" % (v,v2,k))	


	for i in N_zero:
		for v in launchesfrom[i]:
			if (REQUIRE_DRIVER):
				# Constraint (90):
				m.addConstr(decvarzl[0][v][i] + decvarzl[v][0][i] + decvarzhat[A[v][i]] == 1, "Constr.90.%d.%d" % (v,i))

			for v2 in launchesfrom[i]:
				if (v2 != v):
					# Constraint (91):
					m.addConstr(decvarzl[v][v2][i] + decvarzl[v2][v][i] <= 1, "Constr.91.%d.%d.%d" % (v,v2,i))

					# Constraint (92):	
					m.addConstr(decvarzl[v][v2][i] + decvarzl[v2][v][i] + decvarzhat[A[v][i]] + decvarzhat[A[v2][i]] >= 1, "Constr.92.%d.%d.%d" % (v,v2,i))	
	

	for k in N_zero:
		if (k != 0):
			for v in landsat[k]:
				for v2 in launchesfrom[k]:
					if (v2 != v):
						# Constraint (94):
						m.addConstr(decvarzprime[v2][v][k] + decvarzdp[v][v2][k] <= 1, "Constr.94.%d.%d.%d" % (v,v2,k))

						# Constraint (93):
						m.addConstr(decvarzprime[v2][v][k] + decvarzdp[v][v2][k] + decvarzhat[B[v][k]] + decvarzhat[A[v2][k]] >= 1, "Constr.93.%d.%d.%d" % (v,v2,k))	


	for k in N_plus:
		if (REQUIRE_TRUCK_AT_DEPOT):
			tmpSL = 0
			for v in launchesfrom[k]:
				tmpSL += sL[v][k]*(1-decvarzhat[A[v][k]])
			tmpSR = 0
			for v in landsat[k]:
				tmpSR += sR[v][k]*(1-decvarzhat[B[v][k]])
			# Constraint (95):
			m.addConstr(decvarhatt[k] >= decvarcheckt[k] + tmpSL + tmpSR, "Constr.95.%d" % (k))

		elif (k != c+1):
			tmpSL = 0
			for v in launchesfrom[k]:
				tmpSL += sL[v][k]*(1-decvarzhat[A[v][k]])
			tmpSR = 0
			for v in landsat[k]:
				tmpSR += sR[v][k]*(1-decvarzhat[B[v][k]])
			# Constraint (95):
			m.addConstr(decvarhatt[k] >= decvarcheckt[k] + tmpSL + tmpSR, "Constr.95.%d" % (k))


	if (REQUIRE_TRUCK_AT_DEPOT):
		for v in launchesfrom[0]:
			# Constraint (86):
			m.addConstr(decvarhatt[0] >= decvarhattprime[v][0], "Constr.86.%d" % (v))	

	
	# Solve
	m.optimize()

	ls_hatt = {}
	ls_checkt = {}
	ls_checktprime = {}
	

	# Need to build the TSP solution in order
	if (m.Status == GRB.INFEASIBLE):
		# NO FEASIBLE SOLUTION
		print "P3 IS INFEASIBLE"

		assignmentsArray 	= []
		packagesArray 		= []
		p3isFeasible 		= False
		p3OFV				= float('inf')
		waitingTruck		= -1
		waitingUAV			= -1
		waitingArray 		= {}					

	elif ((m.Status == GRB.TIME_LIMIT) and (m.objVal > 1e30)):
		# NO FEASIBLE SOLUTION WAS FOUND (maybe one exists, but we ran out of time)
		print "P3 Time Limit Reached"
		
		p3isFeasible 			= False
		p3OFV 					= -1
		assignmentsArray 		= {}
		packagesArray 			= {}
		waitingTruck			= -1
		waitingUAV				= -1
		waitingArray 			= {}

	elif (m.Status == GRB.CUTOFF):
		# Lower bound is greater than current incumbent.
		print "P3 Objective bound is worse than CutOff"
		
		p3isFeasible 			= False
		p3OFV 					= -1
		assignmentsArray 		= {}
		packagesArray 			= {}
		waitingTruck			= -1
		waitingUAV				= -1
		waitingArray 			= {}		
	
	else:
		p3isFeasible 			= True
		
		for [v,i,j,k] in y:
			if (decvarzhat[j].x > 0.9):
				# No feasible solution, since customer j has infeasible UAV assignment
				print 'zhat[%d] = 1.0' % (j)
				p3isFeasible 			= False
				break										
		
				
		if (p3isFeasible == False):
			print "P3 IS INFEASIBLE"
			
			p3isFeasible 			= False
			p3OFV 					= -1
			assignmentsArray 		= {}
			packagesArray 			= {}
			waitingTruck			= -1
			waitingUAV				= -1
			waitingArray 			= {}


		else:
			# A feasible solution is FOUND
			print '\nOBJECTIVE FUNCTION VALUE:', m.objVal
	
			p3isFeasible 			= True
			p3OFV 					= m.objVal
			waitingTruck			= 0.0
			waitingUAV				= 0.0
			

			# BUILD ASSIGNMENTS AND PACKAGES DICTIONARIES:

			prevTime = {}
			assignmentsArray = {}
			packagesArray = {}
			waitingArray = {}
			waitingArray[0] = 0
			
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

				dummy_1 = 0
				for v in launchesfrom[i]:
					dummy_1 += sL[v][i]

				dummy_2 = 0
				for v in landsat[i]:
					dummy_2 += sR[v][i]

				waitingArray[i] = (decvarcheckt[j].x - decvarcheckt[i].x) - (tau[i][j] + sigma[i] + dummy_1 + dummy_2)
				
				# Are there any UAVs on the truck when the truck leaves i?
				for v in V:
					if ((v in landsat[i]) and (v not in uavRiders)):
						uavRiders.append(v)
					if ((v in launchesfrom[i]) and (v in uavRiders)):
						uavRiders.remove(v)
		
		
				# These activities need to be sorted by time (ascending)
				tmpTimes = []
							
				if (i == 0):
					# Is there idle time before leaving the depot?			
					if (abs(decvarhatt[i].x - prevTime[1]) > 0.001):

						# The truck was delayed in departing this node.
						waitingArray[0] = abs(decvarhatt[i].x - prevTime[1])
						
						if (len(uavRiders) > 0):
							A_statusID = STATIONARY_TRUCK_W_UAV
						else:
							A_statusID = STATIONARY_TRUCK_EMPTY
						A_vehicleType = TYPE_TRUCK
						A_startTime = 0.0
						A_startNodeID = i
						A_startLatDeg = node[i].latDeg
						A_startLonDeg = node[i].lonDeg
						A_startAltMeters = 0.0
						A_endTime = decvarhatt[i].x
						A_endNodeID = i
						A_endLatDeg = node[i].latDeg
						A_endLonDeg = node[i].lonDeg
						A_endAltMeters = 0.0
						A_icon = tmpIcon
						A_description = 'Idle at depot for %3.0f seconds' % (A_endTime - A_startTime)
						A_UAVsOnBoard = uavRiders
						A_ganttStatus = GANTT_IDLE
				
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
					myMin, mySec	= divmod(decvarbart[j].x, 60)
					myHour, myMin 	= divmod(myMin, 60)
					A_description	= 'Arrived at the Depot.  Total Time = %d:%02d:%02d' % (myHour, myMin, mySec) 
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
		
				if (j < c+1):
					# We're going to ignore UAVs that land at the depot.
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
				waitingUAV += ((decvarchecktprime[v][k].x - decvarcheckt[i].x) - (tauprime[v][i][j] + tauprime[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))
				
				waitingArray[j] = ((decvarchecktprime[v][k].x - decvarcheckt[i].x) - (tauprime[v][i][j] + tauprime[v][j][k] + sigmaprime[j] + sL[v][i] + sR[v][k]))
				
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


			# Capture the values of hatt, checkt and checktprime for use in local search:
			for k in N:
				if k not in C:
					ls_hatt[k] = decvarhatt[k].x
					ls_checkt[k] = decvarcheckt[k].x
				for v in V:
					if ((v in landsat[k]) or (v in launchesfrom[k])):
						if (k != 0):
							ls_checktprime[v,k] = decvarchecktprime[v][k].x


	return (p3isFeasible, p3OFV, assignmentsArray, packagesArray, waitingTruck, waitingUAV, waitingArray, landsat, launchesfrom, ls_checkt, ls_hatt, ls_checktprime)