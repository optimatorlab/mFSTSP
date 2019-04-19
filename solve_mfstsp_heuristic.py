#!/usr/bin/env python

import sys
import time
import datetime
import math
from parseCSV import *
from gurobipy import *
from collections import defaultdict
import copy

import os

from mfstsp_heuristic_1_partition import *
from mfstsp_heuristic_2_asgn_uavs import *
from mfstsp_heuristic_3_timing import *

from local_search import *

import endurance_calculator

import random

# 
NODE_TYPE_DEPOT	= 0
NODE_TYPE_CUST	= 1

TYPE_TRUCK 		= 1
TYPE_UAV 		= 2
#


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


# Function to generate TSP assignments for a given TSP tour:
def generateTSPinfo(myTour, c, C, node, tau, sigma):
	tmpAssignments = defaultdict(make_dict)

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
			print 'WE HAVE A PROBLEM.  What is the proper description?'
			print '\t Quitting Now.'
			exit()

		if (0 in tmpAssignments[1][statusID]):
			statusIndex = len(tmpAssignments[1][statusID])
		else:
			statusIndex = 0
		
		tmpAssignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)


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
			description		= 'Arrived at the Depot.'
		else:
			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_DELIVER
			description		= 'Dropping off package to Customer %d' % (j)

		if (0 in tmpAssignments[1][statusID]):
			statusIndex = len(tmpAssignments[1][statusID])
		else:
			statusIndex = 0
		
		tmpAssignments[1][statusID][statusIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)

		tmpDepart = endTime
		if (j != c+1):
			# Go to the next arc
			i = j

	return (objVal, tmpAssignments, myTour)	
	
	
# Function to insert a UAV customer myj between two truck customers myi and myk, and obtain TSP assignments for the resulting TSP tour:		
def insertTruckCustomer(myj, myi, myk, c, C, node, tau, sigma, x):
	tmpAssignments = defaultdict(make_dict)
	
	vehicleType = TYPE_TRUCK
	UAVsOnBoard = []
	startAltMeters = 0.0
	endAltMeters = 0.0
	tmpDepart = 0.0
	icon = 'ub_truck_1.gltf'

	tmpDepart = 0.0
	
	vehicleID = 1	# Truck
	
	TSPobjVal = 0.0
	
	tmpTSPtour = []
		
	for [i1, j1] in x:
		if ((i1 == myi) and (j1 == myk)):
			# Insert myj between myi and myk:

			subtour = [[myi, myj], [myj, myk]]
		else:
			# Travel from i1 to j1
			
			subtour = [[i1, j1]]	
		
		for [i,j] in subtour:
			
			tmpTSPtour.append(i)

			# We are traveling from i to j
			# Capture the "traveling" component:
			statusID 	= TRAVEL_TRUCK_EMPTY
			ganttStatus	= GANTT_TRAVEL
			startTime 	= tmpDepart		# When we departed from i
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
				print 'WE HAVE A PROBLEM.  What is the proper description?'
				print '\t Quitting Now.'
				exit()

			if (0 in tmpAssignments[1][statusID]):
				myIndex = len(tmpAssignments[1][statusID])
			else:
				myIndex = 0

			TSPobjVal = max(TSPobjVal, endTime)
			
			tmpAssignments[1][statusID][myIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)
		

			# Now, capture the "service" component at myj:
			startTime 		= endTime		# When we arrived at j
			startNodeID 	= j
			startLatDeg 	= node[j].latDeg
			startLonDeg		= node[j].lonDeg
			endTime			= startTime + sigma[j]	# This is when we finish up at j
			endNodeID		= j
			endLatDeg		= node[j].latDeg
			endLonDeg		= node[j].lonDeg

			statusID		= STATIONARY_TRUCK_EMPTY
			ganttStatus		= GANTT_DELIVER
			description		= 'Dropping off package to Customer %d' % (j)
			
			if (0 in tmpAssignments[1][statusID]):
				myIndex = len(tmpAssignments[1][statusID])
			else:
				myIndex = 0

			TSPobjVal = max(TSPobjVal, endTime)

			tmpAssignments[1][statusID][myIndex] = make_assignments(vehicleType, startTime, startNodeID, startLatDeg, startLonDeg, startAltMeters, endTime, endNodeID, endLatDeg, endLonDeg, endAltMeters, icon, description, UAVsOnBoard, ganttStatus)
	
			tmpDepart = endTime
	
	tmpTSPtour.append(c+1)
	
	return (tmpAssignments, TSPobjVal, tmpTSPtour)


# Function to find a truck customer to move to a UAV such that best reduction in make-span can be obtained:
def ImproveMakeSpan(c, myTSPtour, y, waitingArray, customersTruck, customersUAV, V, sL, sR, tau, tauprime, sigma, sigmaprime, node, vehicle, eee, prevTSPtours):
	
	if (len(myTSPtour) != len(customersTruck) + 2):
		print "len(TSP) = %d" % len(myTSPtour)
		print "len(truck) = %d" % len(customersTruck)
		print myTSPtour
		print customersTruck
		exit()
		
		
	bestSavings = 0
	bestAction = []


	if len(V) < 1:
		availUAVs = {}
		LR = {}

		for tmpIndex in range(0, len(myTSPtour)-1):
			i_launch = myTSPtour[tmpIndex]
			availUAVs[i_launch] = list(V)
			if tmpIndex > 0:
				LR[i_launch] = 0

			for [v,i,j,k] in y:
				p = myTSPtour.index(i)
				q = myTSPtour.index(k) - 1

				if tmpIndex >= p and tmpIndex <= q:
					availUAVs[i_launch] = []
					break

		for [v,i,j,k] in y:
			if i != myTSPtour[0]:
				LR[i] += 1
			if k != myTSPtour[-1]:
				LR[k] += 1


		for tmpIndex in range(0, len(myTSPtour)-2):
			i = myTSPtour[tmpIndex]
			j = myTSPtour[tmpIndex+1]
			k = myTSPtour[tmpIndex+2]

			if LR[j] <= 1:
			
				tmpTSPtour = list(myTSPtour)
				tmpTSPtour.remove(j)

				if (tmpTSPtour not in prevTSPtours):

					continue_outer_loop = False

					savings = tau[i][j] + sigma[j] + tau[j][k] - tau[i][k]

					for [v,iprime,jprime,kprime] in y:
						if (kprime == j):
							p = myTSPtour.index(iprime)
							q = myTSPtour.index(kprime)
							duration_bw_i_j = 0
							for r in range(p,q):
								i_temp = myTSPtour[r]
								j_temp = myTSPtour[r+1]
								duration_bw_i_j += tau[i_temp][j_temp]
								if r != q-1:
									duration_bw_i_j += sigma[j_temp]
							
							duration_bw_i_k = duration_bw_i_j - tau[i][j] + tau[i][k]						
							
							duration_bw_i_j = max(duration_bw_i_j , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])
							
							if (duration_bw_i_k <= eee[v][iprime][jprime][k]) and (tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][k] <= eee[v][iprime][jprime][k]):
								duration_bw_i_k = max(duration_bw_i_k , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][k])
								savings = duration_bw_i_j + sigma[j] + tau[j][k] - duration_bw_i_k
							else:
								continue_outer_loop = True
								break

							break

						if (iprime == j):
							p = myTSPtour.index(iprime)
							q = myTSPtour.index(kprime)
							duration_bw_j_k = 0
							for r in range(p,q):
								i_temp = myTSPtour[r]
								j_temp = myTSPtour[r+1]
								duration_bw_j_k += tau[i_temp][j_temp]
								if r != q-1:
									duration_bw_j_k += sigma[j_temp]
							
							duration_bw_i_k = duration_bw_j_k - tau[j][k] + tau[i][k]						
							
							duration_bw_j_k = max(duration_bw_j_k , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])
							
							if (duration_bw_i_k <= eee[v][i][jprime][kprime]) and (tauprime[v][i][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime] <= eee[v][i][jprime][kprime]):
								duration_bw_i_k = max(duration_bw_i_k , tauprime[v][i][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])
								savings = tau[i][j] + sigma[j] + duration_bw_j_k - duration_bw_i_k
							else:
								continue_outer_loop = True
								break

							break

						if (myTSPtour.index(iprime) < myTSPtour.index(j)) and (myTSPtour.index(kprime) > myTSPtour.index(j)):
							p = myTSPtour.index(iprime)
							q = myTSPtour.index(kprime)
							duration_bw_i_k_before = 0
							for r in range(p,q):
								i_temp = myTSPtour[r]
								j_temp = myTSPtour[r+1]
								duration_bw_i_k_before += tau[i_temp][j_temp]
								if r != q-1:
									duration_bw_i_k_before += sigma[j_temp]

							duration_bw_i_k_after = duration_bw_i_k_before - tau[i][j] - sigma[j] - tau[j][k] + tau[i][k]

							duration_bw_i_k_before = max(duration_bw_i_k_before , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])

							if (duration_bw_i_k_after <= eee[v][iprime][jprime][kprime]) and (tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime] <= eee[v][iprime][jprime][kprime]):
								duration_bw_i_k_after = max(duration_bw_i_k_after , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])
								savings = duration_bw_i_k_before - duration_bw_i_k_after
							else:
								continue_outer_loop = True
								break

							break
							

					if continue_outer_loop == True:
						continue

					# Insert j between i and k such that bestSavings is maximum:
					for i_new_index in range(0, len(tmpTSPtour)-1):
						i_new = tmpTSPtour[i_new_index]
						k_new = tmpTSPtour[i_new_index + 1]

						if i_new == i:
							if (LR[j] == 0) and len(availUAVs[j]) == 1:
								for v in availUAVs[j]:
									if (node[j].parcelWtLbs <= vehicle[v].capacityLbs):
										if (tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] <= eee[v][i][j][k]) and (tau[i][k] <= eee[v][i][j][k]):
											cost = sL[v][i] + sR[v][k] + max(0 , tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] - tau[i][k])

											if savings - cost > bestSavings:
												bestSavings = savings - cost
												bestAction = [i, j, k]

						else:
							if len(availUAVs[i_new]) > 0:
								for v in availUAVs[i_new]:
									if (node[j].parcelWtLbs <= vehicle[v].capacityLbs):
										if (tauprime[v][i_new][j] + sigmaprime[j] + tauprime[v][j][k_new] <= eee[v][i_new][j][k_new]) and (tau[i_new][k_new] <= eee[v][i_new][j][k_new]):
											cost = sL[v][i_new] + sR[v][k_new] + max(0 , tauprime[v][i_new][j] + sigmaprime[j] + tauprime[v][j][k_new] - tau[i_new][k_new])

											if savings - cost > bestSavings:
												bestSavings = savings - cost
												bestAction = [i_new , j, k_new]

	else:
		availUAVs = {}
		duration_bw_nodes = {}
		for tmpIndex in range(0, len(myTSPtour)-1):
			i = myTSPtour[tmpIndex]
			k = myTSPtour[tmpIndex + 1]
			availUAVs[i] = []
			duration_bw_nodes[i,k] = tau[i][k]

		for [v,i,j,k] in y:
			availUAVs[i].append(v)
			duration_bw_nodes[i,k] = max(duration_bw_nodes[i,k] , tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k])

		for tmpIndex in range(0, len(myTSPtour)-1):
			i = myTSPtour[tmpIndex]
			availUAVs[i] = list(set(V) - set(availUAVs[i]))	


		for tmpIndex in range(0, len(myTSPtour)-2):
			i = myTSPtour[tmpIndex]
			j = myTSPtour[tmpIndex+1]
			k = myTSPtour[tmpIndex+2]
			
			tmpTSPtour = list(myTSPtour)
			tmpTSPtour.remove(j)

			if (tmpTSPtour not in prevTSPtours):

				# Can't move j to UAV if j is being used to launch/recover UAVs
						
				# Is there a UAV available to launch from i, serve j, and return to k?
				continue_outer_loop = False
				duration_bw_i_k = tau[i][k]
				LR_at_j = 0
				for [v,iprime,jprime,kprime] in y:
					if (kprime == j):
						LR_at_j += 1						
						if (tau[i][k] <= eee[v][iprime][jprime][k]) and (tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][k] <= eee[v][iprime][jprime][k]):
							duration_bw_i_k = max(duration_bw_i_k , tauprime[v][iprime][jprime] + sigmaprime[jprime] + tauprime[v][jprime][k])
						else:
							continue_outer_loop = True
							break

					if (iprime == j):
						LR_at_j += 1						
						if (tau[i][k] <= eee[v][i][jprime][kprime]) and (tauprime[v][i][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime] <= eee[v][i][jprime][kprime]):
							duration_bw_i_k = max(duration_bw_i_k , tauprime[v][i][jprime] + sigmaprime[jprime] + tauprime[v][jprime][kprime])
						else:
							continue_outer_loop = True
							break

				if LR_at_j > len(V) or continue_outer_loop == True:
					continue

				savings = duration_bw_nodes[i,j] + sigma[j] + duration_bw_nodes[j,k] - duration_bw_i_k

				# Insert j between i and k such that bestSavings is maximum:
				for i_new_index in range(0, len(tmpTSPtour)-1):
					i_new = tmpTSPtour[i_new_index]
					k_new = tmpTSPtour[i_new_index + 1]

					if i_new == i:
						if LR_at_j < len(V):
							for v in list(set(availUAVs[i]).intersection(set(availUAVs[j]))):
								if (node[j].parcelWtLbs <= vehicle[v].capacityLbs):
									if (tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] <= eee[v][i][j][k]) and (tau[i][k] <= eee[v][i][j][k]):
										cost = sL[v][i] + sR[v][k] + max(0 , tauprime[v][i][j] + sigmaprime[j] + tauprime[v][j][k] - duration_bw_i_k)

										if savings - cost > bestSavings:
											bestSavings = savings - cost
											bestAction = [i, j, k]

					else:
						if len(availUAVs[i_new]) > 0:
							for v in availUAVs[i_new]:
								if (node[j].parcelWtLbs <= vehicle[v].capacityLbs):
									if (tauprime[v][i_new][j] + sigmaprime[j] + tauprime[v][j][k_new] <= eee[v][i_new][j][k_new]) and (tau[i_new][k_new] <= eee[v][i_new][j][k_new]):
										cost = sL[v][i_new] + sR[v][k_new] + max(0 , tauprime[v][i_new][j] + sigmaprime[j] + tauprime[v][j][k_new] - duration_bw_nodes[i_new,k_new])

										if savings - cost > bestSavings:
											bestSavings = savings - cost
											bestAction = [i_new , j, k_new]
					
	
	myCustomersTruck = list(customersTruck)
	myCustomersUAV = list(customersUAV)
	myTSPtour = list(myTSPtour)
	
	if (bestSavings > 0):
		# Remove j from truck
		myCustomersTruck.remove(bestAction[1])
		# Add j to UAV
		myCustomersUAV.append(bestAction[1])
		# Remove j from truck tour
		myTSPtour.remove(bestAction[1])
		
	return (bestSavings, bestAction, myTSPtour, myCustomersTruck, myCustomersUAV)


		
def solve_mfstsp_heuristic(node, vehicle, travel, cutoffTime, problemName, problemType, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER):
	
	# Establish system parameters:
	C 			= []
	tau			= defaultdict(make_dict)
	tauprime 	= defaultdict(make_dict)
	eee 		= defaultdict(make_dict) # Endurance (in seconds) of vehicle v if it travels from i --> j --> k
	eeePrime 	= defaultdict(make_dict) # Maximum possible Enduance (in seconds) of vehicle v if it is launched from i and retrieved at k
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
			V.append(vehicleID)
												
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
				eeePrime[vehicleID][i][j] = 0
				if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
					tau[i][j] = travel[vehicleID][i][j].totalTime
					if (tau[i][j] > minDistance):
						minDistance = tau[i][j]
				elif (vehicle[vehicleID].vehicleType == TYPE_UAV):
					tauprime[vehicleID][i][j] = travel[vehicleID][i][j].totalTime
				else:
					print "ERROR:  Vehicle Type %d is not defined." % (vehicle[vehicleID].vehicleType)
					quit()	
					
			# NOTE: We need to capture the travel time to node c+1 (which is the same physical location as node 0):
			eeePrime[vehicleID][i][c+1] = 0
			if (vehicle[vehicleID].vehicleType == TYPE_TRUCK):
				tau[i][c+1] = travel[vehicleID][i][0].totalTime
				if (tau[i][c+1] > minDistance):
					minDistance = tau[i][c+1]
			elif (vehicle[vehicleID].vehicleType == TYPE_UAV):
				tauprime[vehicleID][i][c+1] = travel[vehicleID][i][0].totalTime
			else:
				print "ERROR:  Vehicle Type %d is not defined." % (vehicle[vehicleID].vehicleType)
				quit()	
										
										
	# Build the set of all possible sorties: [v, i, j, k]
	P = []
	for v in V:		# vehicle
		for i in N_zero:
			for j in C:
				if ((j != i) and (node[j].parcelWtLbs <= vehicle[v].capacityLbs)):
					for k in N_plus:
						if (k != i) and (k != j):

							# Calculate the endurance for each sortie:
							if (k == c+1):
								eee[v][i][j][k] = endurance_calculator.give_endurance(node, vehicle, travel, v, i, j, 0)
							else:	
								eee[v][i][j][k] = endurance_calculator.give_endurance(node, vehicle, travel, v, i, j, k)
							eeePrime[v][i][k] = max(eeePrime[v][i][k], eee[v][i][j][k])		# This is only used in Phase 2

							if (tauprime[v][i][j] + node[j].serviceTimeUAV + tauprime[v][j][k] <= eee[v][i][j][k]):
								if (REQUIRE_TRUCK_AT_DEPOT):
									if (tau[i][k] <= eee[v][i][j][k]):
										P.append([v,i,j,k])									
								else:	
									if ((k == c+1) or (tau[i][k] <= eee[v][i][j][k])):
										P.append([v,i,j,k])   # This relaxes the requirement that the truck picks up the UAV from the depot at the end.

	# Build the launch service times:
	for v in V:
		for i in N_zero:
			sL[v][i] = vehicle[v].launchTime
			
	# Build the recovery service times:
	for v in V:
		for k in N_plus:
			sR[v][k] = vehicle[v].recoveryTime

	# Build the customer service times:
	sigma[0] = 0.0
	for k in N_plus:
		if (k == c+1):
			sigma[k] = 0.0
			sigmaprime[k] = 0.0
		else:
			sigma[k] = node[k].serviceTimeTruck	
			sigmaprime[k] = node[k].serviceTimeUAV
		

	# Initialize the variable that keeps track of best objective found so far:
	bestOFV = float('inf')

	
	# Calculate the lower truck limit
	LTLbase = int(math.ceil((float(len(C) - len(V))/float(1 + len(V)))))

	
	prevTSPtours = []	# List to store all the TSPs that we will see in this problem instance


	p1_previousTSP = []
	FEASobjVal = 0


	for LTL in range(LTLbase, c+1):

		tryingP3improvement = False		# Initialize this flag
	
		requireUniqueTSP = True

		#-------------------------------------------------PHASE I STARTS HERE---------------------------------------------------------#

		# Partition customers between truck and UAVs, and generate a unique truck tour:
		[FEASobjVal, customersTruck, customersUAV, TSPobjVal, TSPassignments, TSPpackages, TSPtour, foundTSP, prevTSPtours, p1_previousTSP] = mfstsp_heuristic_1_partition(node, vehicle, travel, N, N_zero, N_plus, C, P, tau, tauprime, sigma, sigmaprime, sL, sR, LTL, requireUniqueTSP, prevTSPtours, bestOFV, p1_previousTSP, FEASobjVal)	


		# If we cant find a unique TSP tour, go back to the start of Phase I with a new LTL:
		if (not foundTSP):
			continue

		# Generate a lower bound:
		optLowBnd = TSPobjVal
		for j in customersUAV:
			v = 2
			optLowBnd += sL[v][j]
			optLowBnd += sR[v][j]
		
		# If lower bound is greater than the current OFV, go back to the start of Phase I with a new LTL:	
		if (optLowBnd >= bestOFV):
			continue
		
		keepTrying2 = True

		while (keepTrying2):
			
			keepTrying2 = False

			#-------------------------------------------------PHASE II STARTS HERE---------------------------------------------------------#

			# Create UAV sorties <v,i,j,k> (a pair of launch-recovery points and a UAV for each UAV customer)
			[insertCost, x, y, z, insertTuple] = mfstsp_heuristic_2_asgn_uavs(node, eee, eeePrime, N_zero, N_plus, C, V, c, TSPassignments, customersUAV, customersTruck, sigma, sigmaprime, tau, tauprime, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, vehicle, sL, sR, prevTSPtours)


			# Generate an optimistic lower bound assuming that truck never waits for UAVs:
			optLowBnd = TSPobjVal
			optLowBnd += insertCost
			for [v,i,j,k] in y:
				if (i != 0):
					optLowBnd += sL[v][i]
				optLowBnd += sR[v][k]
			

			# Check for lower bound:
			if (optLowBnd > bestOFV):	# lower bound is greater than the current OFV, so go back to the start of Phase I with a new LTL
				canDo3 = False
				keepTrying2 = False

			else:	# lower bound is promising, check for Phase II feasibility
				if (len(z) > 0):	# Phase II is not feasible, bceause customer z does not have a feasible assignment
					canDo3 = False

					if (tryingP3improvement):
						# After finding a feasible Phase III solution, we try to move a customer to a UAV in the improvement step.
						# If this isn't immediately feasible, we'll stop trying, and go back to the start of Phase I with a new LTL.
						keepTrying2 = False
						tryingP3improvement = False

					else:
						# Let's try inserting a UAV customer with infeasible assignment in truck route:
						customersTruck.append(z[0])
						customersUAV.remove(z[0])

						# The UAV customer with cheapest insertion cost is z[0], and corresponding insertion location information is stored in insertTuple.
						# Use insertTuple to create a new TSP, and obtain corresponding TSP assignments as following:
						[tmpTSPassignments, tmpTSPobjVal, tmpTSPtour] = insertTruckCustomer(insertTuple['j'], insertTuple['i'], insertTuple['k'], c, C, node, tau, sigma, x)

						if (tmpTSPtour in prevTSPtours):
							# We've already seen this.  No need to re-try
							keepTrying2 = False

						else:
							keepTrying2 = True	
							TSPassignments = tmpTSPassignments
							TSPobjVal = tmpTSPobjVal
							TSPtour = list(tmpTSPtour)
							prevTSPtours.append(TSPtour)

				else:	# Phase II is feasible, and it seems worthwhile to try Phase III
					canDo3 = True					


			if (canDo3):
				
				#-------------------------------------------------PHASE III STARTS HERE---------------------------------------------------------#
				
				# Solve (P3) to determine the schedule of different activities:
				[p3isFeasible, p3objVal, tmpAssignmentsArray, tmpPackagesArray, waitingTruck, waitingUAV, waitingArray, landsat, launchesfrom, ls_checkt, ls_hatt, ls_checktprime] = mfstsp_heuristic_3_timing(x, y, z, node, eee, N, P, V, cutoffTime, c, sigma, sigmaprime, tau, tauprime, minDistance, sR, sL, vehicle, travel, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, optLowBnd)


				# Check Phase III feasibility:
				if (p3isFeasible):

					keepTrying2 = False														
					
					# Update the incumbent:
					if (p3objVal < bestOFV):
						
						assignmentsArray = tmpAssignmentsArray
						packagesArray = tmpPackagesArray
						objVal = p3objVal

						bestWaitingTruck = waitingTruck
						bestWaitingUAV = waitingUAV
						
						bestOFV = p3objVal


					# Improvement Step: (to try to reduce the makespan by moving a truck customer to a UAV)
					[bestSavings, bestAction, tmpTSPtour, tmpCustomersTruck, tmpCustomersUAV] = ImproveMakeSpan(c, TSPtour, y, waitingArray, customersTruck, customersUAV, V, sL, sR, tau, tauprime, sigma, sigmaprime, node, vehicle, eee, prevTSPtours)
					tryingP3improvement = False

					if (bestSavings > 0):	# If saving is possible, make the move and go back to Phase II
						keepTrying2 = True
						tryingP3improvement = True
						
						customersTruck = list(tmpCustomersTruck)
						customersUAV = list(tmpCustomersUAV)

						# Create TSP assignments using the new TSP tour:
						[TSPobjVal, TSPassignments, TSPtour] = generateTSPinfo(tmpTSPtour, c, C, node, tau, sigma)
						prevTSPtours.append(TSPtour)


					else:	# Perform local search (try shifting retrieval points for UAVs to the next location, if the truck waits at the current retrieval location)
						while (True):
							[shift_happened, tmp_y] = local_search(x, y, c, waitingArray, landsat, launchesfrom, ls_checktprime, eee, tau, tauprime, sigma, sigmaprime, ls_checkt, ls_hatt, V, sL, sR)

							if shift_happened == True: # Shift is possible. Therefore re-solve (P3) after making those shifts, and obtain new solution
								[p3isFeasible, p3objVal, tmpAssignmentsArray, tmpPackagesArray, waitingTruck, waitingUAV, waitingArray, landsat, launchesfrom, ls_checkt, ls_hatt, ls_checktprime] = mfstsp_heuristic_3_timing(x, tmp_y, z, node, eee, N, P, V, cutoffTime, c, sigma, sigmaprime, tau, tauprime, minDistance, sR, sL, vehicle, travel, REQUIRE_TRUCK_AT_DEPOT, REQUIRE_DRIVER, optLowBnd)

								# Check Phase III feasibility:
								if (p3isFeasible):	# Phase III is feasible. Update the incumbent, and go back to local search

									y = []
									for [v,i,j,k] in tmp_y:
										y.append([v,i,j,k])

									keepTrying2 = False														
									
									# Update the incumbent:
									if (p3objVal < bestOFV):
										
										assignmentsArray = tmpAssignmentsArray
										packagesArray = tmpPackagesArray
										objVal = p3objVal

										bestWaitingTruck = waitingTruck
										bestWaitingUAV = waitingUAV
										
										bestOFV = p3objVal

								else:	# Phase III is infeasible. Go back to the start of Phase I with a new LTL.
									break
								
							else:	# Shift is not possible. Go back to the start of Phase I with a new LTL.
								break

				else:
					# Phase III is infeasible. Go back to the start of Phase I with a new LTL.
					keepTrying2 = False
			# End Phase III loop
		# End keep trying Phase II loop
	# End LTL loop
	
	# Convert best solution to "assignments" and "packages" classes
	packages = {}
	assignments = defaultdict(make_dict)
	
	for j in packagesArray:
		packages[j] = make_packages(packagesArray[j][0], packagesArray[j][1], packagesArray[j][2], packagesArray[j][3], packagesArray[j][4])

	for v in vehicle:
		for i in range(0, len(assignmentsArray[v])):
			statusID = assignmentsArray[v][i][0]
			if (0 in assignments[v][statusID]):
				statusIndex = len(assignments[v][statusID])
			else:
				statusIndex = 0
			
			assignments[v][statusID][statusIndex] = make_assignments(assignmentsArray[v][i][1], assignmentsArray[v][i][2], assignmentsArray[v][i][3], assignmentsArray[v][i][4], assignmentsArray[v][i][5], assignmentsArray[v][i][6], assignmentsArray[v][i][7], assignmentsArray[v][i][8], assignmentsArray[v][i][9], assignmentsArray[v][i][10], assignmentsArray[v][i][11], assignmentsArray[v][i][12], assignmentsArray[v][i][13], assignmentsArray[v][i][14], assignmentsArray[v][i][15])

		
	return(objVal, assignments, packages, bestWaitingTruck, bestWaitingUAV)