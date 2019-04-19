def local_search(x, y, c, waitingArray, landsat, launchesfrom, ls_checktprime, eee, tau, tauprime, sigma, sigmaprime, ls_checkt, ls_hatt, V, sL, sR):

	tmpWaitingArray = {}
	for i in waitingArray:
		tmpWaitingArray[i] = float(waitingArray[i])

	tmp_y = []
	launchLoc = {}
	custLoc = {}
	for [v,i,j,k] in y:
		tmp_y.append([v,i,j,k])
		launchLoc[v,k] = i
		custLoc[v,k] = j

	TSPtour = []
	for [i,j] in x:
		TSPtour.append(i)
	TSPtour.append(c+1)

	availUAVs = {}

	for tmpIndex in range(0, len(TSPtour)-1):
		i_launch = TSPtour[tmpIndex]
		availUAVs[i_launch] = list(V)

		for [v,i,j,k] in y:
			p = TSPtour.index(i)
			q = TSPtour.index(k) - 1

			if tmpIndex >= p and tmpIndex <= q:
				availUAVs[i_launch].remove(v)

	shift_happened = False

	for truckCust in TSPtour[1:-1]:

		if (tmpWaitingArray[truckCust] > 0) and (len(landsat[truckCust]) > 0):

			tmp_waiting = 0

			for v in landsat[truckCust]:
				if tmp_waiting < ls_checktprime[v,truckCust]:
					tmp_waiting = ls_checktprime[v,truckCust]
					tmpv = v

			tmp_waiting = 0

			for v in landsat[truckCust]:
				if v != tmpv:
					if tmp_waiting < ls_checktprime[v,truckCust]:
						tmp_waiting = ls_checktprime[v,truckCust]
						tmpv_2 = v

			tmpi = launchLoc[tmpv,truckCust]
			tmpj = custLoc[tmpv,truckCust]
			tmpk = TSPtour[TSPtour.index(truckCust)+1]

			if (tauprime[tmpv][tmpi][tmpj] + sigmaprime[tmpj] + tauprime[tmpv][tmpj][tmpk] <= eee[tmpv][tmpi][tmpj][tmpk]):
				if len(landsat[truckCust]) > 1:
					truck_duration_from_i_to_k = (ls_checkt[tmpk] - ls_hatt[tmpi]) - (ls_checktprime[tmpv,truckCust] - ls_checktprime[tmpv_2,truckCust])
				else:
					truck_duration_from_i_to_k = (ls_checkt[tmpk] - ls_hatt[tmpi]) - (ls_checktprime[tmpv,truckCust] - ls_checkt[truckCust] - sigma[truckCust])

				if truck_duration_from_i_to_k <= eee[tmpv][tmpi][tmpj][tmpk]:
					if (tmpv not in launchesfrom[truckCust]):
						tmp_y.remove([tmpv,tmpi,tmpj,truckCust])
						tmp_y.append([tmpv,tmpi,tmpj,tmpk])
						# del launchLoc[tmpv,truckCust]
						# del custLoc[tmpv,truckCust]
						# launchLoc[tmpv,tmpk] = tmpi
						# custLoc[tmpv,tmpk] = tmpj
						if tmpk != c+1:
							tmpWaitingArray[tmpk] -= sR[tmpv][tmpk]
						shift_happened = True

					elif (tmpv in launchesfrom[truckCust]) and (len(availUAVs[truckCust]) >= 1):
						tmp_y.remove([tmpv,tmpi,tmpj,truckCust])
						tmp_y.append([tmpv,tmpi,tmpj,tmpk])
						# del launchLoc[tmpv,truckCust]
						# del custLoc[tmpv,truckCust]
						# launchLoc[tmpv,tmpk] = tmpi
						# custLoc[tmpv,tmpk] = tmpj
						if tmpk != c+1:
							tmpWaitingArray[tmpk] -= sR[tmpv][tmpk]
						shift_happened = True

						tmp_again_y = []
						for [v,i,j,k] in tmp_y:
							tmp_again_y.append([v,i,j,k])

						tmp_again_y_1 = []

						launchesfrom_temp = {}
						landsat_temp = {}
						for i in launchesfrom:
							launchesfrom_temp[i] = []
							landsat_temp[i] = []

						ls_checktprime_temp = {}

						new_v = availUAVs[truckCust][0]

						for [v,i,j,k] in tmp_y:
							if (v == tmpv) and (TSPtour.index(i) >= TSPtour.index(truckCust)):
								tmp_again_y.remove([v,i,j,k])
								tmp_again_y_1.append([new_v,i,j,k])
								launchesfrom[i].remove(v)
								launchesfrom_temp[i].append(new_v)
								landsat[k].remove(v)
								landsat_temp[k].append(new_v)
								ls_checktprime_temp[new_v,k] = ls_checktprime[v,k]
								del ls_checktprime[v,k]

							if (v == new_v) and (TSPtour.index(i) >= TSPtour.index(truckCust)):
								tmp_again_y.remove([v,i,j,k])
								tmp_again_y_1.append([tmpv,i,j,k])
								launchesfrom[i].remove(v)
								launchesfrom_temp[i].append(tmpv)
								landsat[k].remove(v)
								landsat_temp[k].append(tmpv)
								ls_checktprime_temp[tmpv,k] = ls_checktprime[v,k]
								del ls_checktprime[v,k]

						tmp_y = []
						launchLoc = {}
						custLoc = {}
						for [v,i,j,k] in tmp_again_y:
							tmp_y.append([v,i,j,k])
							launchLoc[v,k] = i
							custLoc[v,k] = j
						for [v,i,j,k] in tmp_again_y_1:
							tmp_y.append([v,i,j,k])
							launchLoc[v,k] = i
							custLoc[v,k] = j

						for i in launchesfrom_temp:
							launchesfrom[i] = launchesfrom[i] + launchesfrom_temp[i]
							landsat[i] = landsat[i] + landsat_temp[i]

						for [v,k] in ls_checktprime_temp:
							ls_checktprime[v,k] = ls_checktprime_temp[v,k]

						for tmpIndex in range(0, len(TSPtour)-1):
							i = TSPtour[tmpIndex]
							if tmpIndex > TSPtour.index(truckCust):
								if (tmpv in availUAVs[i]) and (new_v not in availUAVs[i]):
									availUAVs[i].append(new_v)
									availUAVs[i].remove(tmpv)
								elif (tmpv not in availUAVs[i]) and (new_v in availUAVs[i]):
									availUAVs[i].append(tmpv)
									availUAVs[i].remove(new_v)															


	return [shift_happened, tmp_y]