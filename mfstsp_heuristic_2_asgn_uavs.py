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

# There's a package color that corresponds to the VEHICLE that delivered the package.
# Right now we only have 5 boxes (so we can have at most 5 trucks).
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


def mfstsp_heuristic_2_asgn_uavs(node, eee, eeePrime, N_zero, N_plus, C, V, c, assignments, customersUAV, customersTruck, sigma, sigmaprime, tau, tauprime, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, vehicle, sL, sR, prevTSPtours):

	# Build some helper dictionaries to (hopefully) make the algorithm faster:
	helper2 = defaultdict(make_dict) 	# helper2[v][j] = [[i,k], ...]
	helper3 = defaultdict(make_dict)	# helper3[v][i] = [[j,k], ...]
	helper4 = defaultdict(make_dict)	# helper4[v][k] = [[i,j], ...]
	helper5 = {}						# helper5[i]    = [[v,j,k], ...]
	helper6 = {}						# helper6[k]    = [[v,i,j], ...]
	helper7 = {}						# helper7[v] 	= [[i,k], ...]
	helper8 = defaultdict(make_dict)	# helper8[v][i][k] = [j, ...]

	UAVCustSupport = {}
	availUAVs = {}

		
	# Get the truck arrival times and build the ordered list of truck visits.
	t = {}	
	myX = []
	myOrderedList = []
	myOrderedList.append(0)	# Initialize with depot.
	for vehicleID in assignments:
		for statusID in assignments[vehicleID]:
			for asgnIndex in assignments[vehicleID][statusID]:
				if (assignments[vehicleID][statusID][asgnIndex].vehicleType == TYPE_TRUCK):		
					if (statusID in [TRAVEL_TRUCK_W_UAV, TRAVEL_TRUCK_EMPTY]):
						i = assignments[vehicleID][statusID][asgnIndex].startNodeID
						j = assignments[vehicleID][statusID][asgnIndex].endNodeID
						
						if (j not in customersTruck):
							# When we solve the TSP, we only use a subset of nodes.  
							# This causes the TSP solution to return a final node that's not c+1.
							j = c+1
							bigM = assignments[vehicleID][statusID][asgnIndex].endTime

						myX.append([i,j])
						if i == 0:
							t[i] = assignments[vehicleID][statusID][asgnIndex].startTime
						t[j] = assignments[vehicleID][statusID][asgnIndex].endTime
						
						if (j in C):
							myOrderedList.append(j)	
	myOrderedList.append(c+1)
	
	# Reset N_zero -- Should only include customers visted by the truck, plus depot 0.	
	# Reset N_plus -- Should only include customers visted by the truck, plus depot c+1.	
	N_zero = []
	N_plus = []
	N_zero.append(0)
	for i in customersTruck:
		N_zero.append(i)
		N_plus.append(i)
	
	N_plus.append(c+1)
	
	# Reset C -- Should only include customers NOT visited by the truck
	C = []
	for i in customersUAV:
		C.append(i)
		
	
	# Find the cost of inserting truck customers:
	insertCost = {}
	insertPairs = {}
	for j in C:
		insertCost[j] = float('inf')
		for [i,k] in myX:
			tmpCost = max(0, (tau[i][j] + sigma[j] + tau[j][k] - tau[i][k]))
			
			if (tmpCost < insertCost[j]):
				insertCost[j] = tmpCost	
				insertPairs[j] = [i, k]

	
	for v in V:
		for j in C:
			helper2[v][j] = []
		for i in N_zero:
			helper3[v][i] = []
		for k in N_plus:
			helper4[v][k] = []

	for i in N_zero:
		helper5[i] = []
	for k in N_plus:
		helper6[k] = []

	for i in N_zero:
		availUAVs[i] = list(V)

	for j in C:
		UAVCustSupport[j] = []

	#-------------------------------------------------ALGORITHM 4 STARTS HERE---------------------------------------------------------#

	# Re-calculate <v, i, j, k> = Pprime, based on TSP solution
	Pprime = []

	for v in V:
		helper7[v] = []
		for tmpi in range(0,len(myOrderedList)-1):
			i = myOrderedList[tmpi]
			# truckStartTime = t[myOrderedList[0]]
			doTruckTiming = True
			if ((i == 0) and (not REQUIRE_DRIVER)):
				doTruckTiming = False	# We're launching from depot, but we don't need the truck/driver	
			tmpkUpper = min(tmpi+2, len(myOrderedList))		
			for tmpk in range(tmpi+1,tmpkUpper):	
				k = myOrderedList[tmpk]
				if ((k == c+1) and (not REQUIRE_TRUCK_AT_DEPOT)):
					doTruckTiming = False	# We're returning to the depot, but we don't need the truck/driver
				if ((doTruckTiming) and (t[k] - t[i] - sigma[i] > eeePrime[v][i][k])):
					break	# exit out of k loop
				helper7[v].append([i,k])
				helper8[v][i][k] = []				
				for j in customersUAV:
							
					if (tauprime[v][i][j] + node[j].serviceTimeUAV + tauprime[v][j][k] <= eee[v][i][j][k]) and (t[k] - t[i] - sigma[i] < eee[v][i][j][k]):					
						helper2[v][j].append([i,k])
						helper3[v][i].append([j,k])
						helper4[v][k].append([i,j])
						helper5[i].append([v,j,k])
						helper6[k].append([v,i,j])
						helper8[v][i][k].append(j)

						UAVCustSupport[j].append([v,i,k]) # List of potential sorties for each UAV customer


	myY = []	# List of UAV sorties obtained at the end of Algorithm 4

	bigZ = []

	SortedCust = []

	# Sort UAV customers in the ascending order of number of potential sorties they have:
	for j in sorted(UAVCustSupport, key=lambda j: len(UAVCustSupport[j])):
		SortedCust.append(j)


	# Create UAV sorties for each UAV customer (lines 14-32 in Algorithm 4):
	for j in SortedCust:

		Waiting = bigM

		sortie = []

		# Lines 19-25 in Algorithm 4:
		for [v,i,k] in UAVCustSupport[j]:
			tempp = myOrderedList.index(i)
			tempq = myOrderedList.index(k)
			availability = True
			for tempindex in range(tempp,tempq):
				if v not in availUAVs[myOrderedList[tempindex]]:
					availability = False
					break

			if availability == True:
				tempWaiting = (tauprime[v][i][j] + node[j].serviceTimeUAV + tauprime[v][j][k]) - (t[k] - t[i])

				if tempWaiting >= 0:
					if tempWaiting < Waiting:
						Waiting = tempWaiting
						sortie = [v,i,j,k]

				else:
					if Waiting >= 0:
						Waiting = tempWaiting
						sortie = [v,i,j,k]
					else:
						if Waiting < tempWaiting:
							Waiting = tempWaiting
							sortie = [v,i,j,k]

		# If no sortie is found for customer j, append it to the list bigZ:
		if len(sortie) == 0:
			bigZ.append(j)

		else:
			myY.append(sortie)
			tempp = myOrderedList.index(sortie[1])
			tempq = myOrderedList.index(sortie[3])


			for tempindex in range(tempp,tempq):
				availUAVs[myOrderedList[tempindex]].remove(sortie[0])


	#-------------------------------------------------ALGORITHM 5 STARTS HERE---------------------------------------------------------#

	myZ = []
	myW = defaultdict(make_dict)
		
	# If there are infeasible customers (len(bigZ) > 0),				
	# we're only going to return only one bigZ customer with the lowest insertCost (lines 33-37 in Algorithm 5)
	insertTuple = {}

	if (len(bigZ) == 0):	# No infeasible customers
		myInsertCost = 0
		# print("len(bigZ) = 0")
	else:	# Some infeasible UAV customers

		# print("len(bigZ) = %d" % len(bigZ))

		bestCost = float('inf')
		bestIPcombo = []

		support = {}	# Keep track of UAV customers that, if inserted, would allow j to be served.
		for j in bigZ:
			support[j] = set()

		for i in C:
			for p in range(1, len(myOrderedList)):
				# Create a TSP route by inserting UAV customer i into the input TSP tour:
				tmpRoute = myOrderedList[0:p] + [i] + myOrderedList[p:len(myOrderedList)]
				
				# Is this TSP route unique?
				if (tmpRoute not in prevTSPtours):
				
					cInsert = tau[myOrderedList[p-1]][i] + tau[i][myOrderedList[p]] - tau[myOrderedList[p-1]][myOrderedList[p]]
					if (i not in bigZ):
						cInsert = cInsert*1.5
						
					cWait = 0.0
					cFail = 0.0
					
					tmpFailures = list(bigZ)
					for j in bigZ:
						isFeas = False
						tmp = float('inf')
						
						if (j == i):
							# We're just going to insert j into 
							isFeas = True
							tmp = 0.0
						else:
							# Could we launch from i?
							truckTime = 0.0		# We'll find the total time to travel from i to some changing k
							iprime = i
							prevk = 0
							for pprime in range(p, len(myOrderedList)):
								k = myOrderedList[pprime]
								truckTime += tau[iprime][k]		# Time to travel from iprime to k
								iprime = k
								if (prevk):
									truckTime += sigma[prevk]	# Time to serve intermediate customers
								prevk = k
								
								if ((tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] <= eee[v][i][j][k]) and (truckTime <= eee[v][i][j][k])):
									isFeas = True
									
									if (max(0, tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] - truckTime) < tmp):
										tmp = max(0, tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] - truckTime)
										
							# Could we launch from k?
							truckTime = 0.0		# We'll find the total time to travel from some changing k to i
							iprime = i
							prevk = 0						
							for pprime in range(p-1, 0-1, -1):
								k = myOrderedList[pprime]
								truckTime += tau[k][iprime]
								iprime = k
								if (prevk):
									truckTime += sigma[prevk]
								prevk = k
															
								if ((tauprime[v][k][j] + sigmaprime[j] + tauprime[v][j][i] <= eee[v][k][j][i]) and (truckTime <= eee[v][k][j][i])):
									isFeas = True
									
									if (max(0, tauprime[v][k][j] + sigmaprime[j] + tauprime[v][j][i] - truckTime) < tmp):
										tmp = max(0, tauprime[v][k][j] + sigmaprime[j] + tauprime[v][j][i] - truckTime)
	
							# Update costs
							if (isFeas):
								cWait += tmp
								cFail += 0.0
								tmpFailures.remove(j)
								support[j].add(i)
							else:
								cWait += 0.0
								cFail += insertCost[j]
	
					if (cInsert + cWait + cFail < bestCost):
						bestCost = cInsert + cWait + cFail
						bestIPcombo = [i, p]
						bestTour = list(tmpRoute)
						failures = list(tmpFailures)

		# We're going to insert customer j in the truck's route between customers i and k:
		p = bestIPcombo[1]		# Position
		j = bestIPcombo[0]		# Inserted customer
		i = myOrderedList[p-1]	# Customer before j
		k = myOrderedList[p]	# Customer after j
		
		myInsertCost = insertCost[j]
		insertTuple = {'j': j, 'i': i, 'k': k}
		myZ = [j]


	return (myInsertCost, myX, myY, myZ, insertTuple)