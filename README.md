LINK TO THE PAPER ON multiple flying sidekick traveling salesman problem (mFSTSP):

https://papers.ssrn.com/sol3/papers.cfm?abstract_id=3338436



INSTALLING ADDITIONAL PYTHON MODULES:

pandas:
	sudo pip install pandas

scipy:
	sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas

		DID NOT DO THESE:
			sudo apt-get install python-numpy python-scipy
			sudo python -m pip install --upgrade pip
			sudo pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose

geopy
	sudo pip install geopy

R TSP
	R		[NOTE:  This was ono version 3.0.2]
	> install.packages("TSP")
	[agree to install in another location (defaults to /home/murray/R/x86_64-pc-linux-gnu-library/3.0)
	I chose CA (ON) as the mirror.  Some mirrors don't have the TSP package. 



RUNNING THIS SCRIPT:

python mfstsp_mission_control.py <city> <problemName> <vehicleFileID> <cutoffTime> <problemType> <numUAVs> <numTrucks> <requireTruckAtDepot> <requireDriver>

city: 'buffalo' or 'seattle'
problemName: Name of the folder containing the data for a particular problem instance
vehicleFileID: 101, 102, 103, 104 (Chooses a particular UAV type depending on the file ID)
cutoffTime: Gurobi cut-off time of IP model. In case of heuristic, it is the Gurobi cut-off time of (P3) model
problemType: 1 (mFSTSP IP) or 2 (mFSTSP Heuristic)
numUAVs: Number of UAVs available in the problem
numTrucks: Number of trucks available in the problem (currently only solvable for 1 truck)
requireTruckAtDepot:  0 (false) or 1 (true).
requireDriver: 0 (false) or 1 (true). False --> UAVs can launch/land without driver (so driver can serve customer).


1) Solving the mFSTSP optimally:
   python mfstsp_mission_control.py seattle 20170608T121632668184 101 3600 1 3 -1 1 1
		numTrucks is ignored
		The UAVs are defined in tbl_vehicles.  If you ask for more UAVs than are defined, you'll get a warning.

2) Solving the mFSTSP via a heuristic:
   python mfstsp_mission_control.py seattle 20170608T121632668184 101 5 2 3 -1 1 1
		numTrucks is ignored
		The UAVs are defined in tbl_vehicles.  If you ask for more UAVs than are defined, you'll get a warning.