from gurobipy import *

def checkP2Feasibility(UAVPossibleCust, TSP, V, Pprime):

	if len(UAVPossibleCust) == 0:
		myStatus = 1
		infeasibleUAVcust = []

	else:
		launch_points = {}
		launch_uav_cust = {}

		for j in UAVPossibleCust:
			launch_points[j] = []

		for iii in range(0, len(TSP)-1):
			i = TSP[iii]
			launch_uav_cust[i] = []
		
		for j in UAVPossibleCust:

			for iii in range(0, len(TSP)-1):
				i = TSP[iii]
				k = TSP[iii+1]

				for v in V:

					if ([i,k] in Pprime[v][j]):
						if i not in launch_points[j]:
							launch_points[j].append(i)
						if j not in launch_uav_cust[i]:
							launch_uav_cust[i].append(j)

		m = Model("Check_Feas")

		# Tell Gurobi not to print to a log file
		m.params.OutputFlag = 0

		x = {} #DecVar

		for i in launch_uav_cust:
			for j in launch_uav_cust[i]:
				x[i,j] = m.addVar(lb=0, ub=1, obj=1, vtype=GRB.BINARY, name="x.%d.%d" % (i,j))

		m.modelSense = GRB.MINIMIZE

		m.params.SolutionLimit = 1

		m.update()

		for i in launch_uav_cust:
			m.addConstr(quicksum(x[i,j] for j in launch_uav_cust[i]) <= len(V), name="Constr.1.%d.%d" % (i,j))

		for j in launch_points:
			m.addConstr(quicksum(x[i,j] for i in launch_points[j]) == 1, name="Constr.2.%d.%d" % (i,j))

		m.optimize()

		infeasibleUAVcust = []

		if (m.Status == GRB.INFEASIBLE):
			myStatus = 0

			# print('The model is infeasible; computing IIS')
			m.computeIIS()

			for q in m.getConstrs():
				if q.IISConstr:
					tmp = q.constrName.split('.')
					if (tmp[1] == '2'):
						j = int(tmp[3])
						infeasibleUAVcust.append(j)

		else:
			myStatus = 1

	return [myStatus, infeasibleUAVcust]