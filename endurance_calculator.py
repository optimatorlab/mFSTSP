from collections import defaultdict

def make_dict():
	return defaultdict(make_dict)

	# Usage:
	# tau = defaultdict(make_dict)
	# v = 17
	# i = 3
	# j = 12
	# tau[v][i][j] = 44

def give_endurance(node, vehicle, travel, v, i, j, k):

	if vehicle[v].flightRange == 'low':
		d = 5.5		# Power consumed (while cruising) per unit payroll mass per unit speed (W/(lb)/(m/s))
		e = 11		# Power consumed (while cruising) by empty vehicle per unit speed (W/(m/s))
		dp = 11		# Power consumed (while landing or takeoff) per unit payroll mass per unit speed (W/(lb)/(m/s))
		ep = 22		# Power consumed (while landing or takeoff) by empty vehicle per unit speed (W/(m/s))
		f = 200		# Power consumed while serving the customer (W)
		g = 400		# Power consumed while waiting to land (W)
	elif vehicle[v].flightRange == 'high':
		d = 5.5		# Power consumed (while cruising) per unit payroll mass per unit speed (W/(lb)/(m/s))
		e = 12		# Power consumed (while cruising) by empty vehicle per unit speed (W/(m/s))
		dp = 11		# Power consumed (while landing or takeoff) per unit payroll mass per unit speed (W/(lb)/(m/s))
		ep = 24		# Power consumed (while landing or takeoff) by empty vehicle per unit speed (W/(m/s))
		f = 225		# Power consumed while serving the customer (W)
		g = 450		# Power consumed while waiting to land (W)
	else:
		print "ERROR: Vehicle range input is not valid."
		exit()	

	# endurance = defaultdict(make_dict)

	# for [v,i,j,k] in P:
	p = vehicle[v].takeoffSpeed
	q = vehicle[v].cruiseSpeed
	r = vehicle[v].landingSpeed
	TTvij = travel[v][i][j].takeoffTime
	FTvij = travel[v][i][j].flyTime
	LTvij = travel[v][i][j].landTime
	# print "[%d][%d][%d] " % (v, j, k)
	# print "\t travel[%d][%d][%d].takeoffTime = %f" % (v, j, k, travel[v][j][k].takeoffTime)
	TTvjk = travel[v][j][k].takeoffTime
	FTvjk = travel[v][j][k].flyTime
	LTvjk = travel[v][j][k].landTime
	sj = node[j].serviceTimeUAV
	mj = node[j].parcelWtLbs
	E = vehicle[v].batteryPower

	minimum_time_required = TTvij + FTvij + LTvij + sj + TTvjk + FTvjk + LTvjk
	minimum_energy_required = (TTvij*p + LTvij*r)*(dp*mj+ep) + FTvij*q*(d*mj+e) + sj*f + (TTvjk*p + LTvjk*r)*ep + FTvjk*q*e
	energy_left = E - minimum_energy_required

	if energy_left >= 0:		
		endurance = minimum_time_required + float(energy_left)/g
		# endurance[v][i][j][k] = minimum_time_required + float(energy_left)/g
	else:
		endurance = -1
		# endurance[v][i][j][k] = -1
		#endurance[v][i][j][k] = E*minimum_time_required/float(minimum_energy_required)

	# if (endurance > 0):
	# 	print "yo", endurance
	
	return endurance
	# return 1200
