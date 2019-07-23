# Source codes for mFSTSP IP and Heuristic

Runs on **Windows / Linux / Mac**.

Compatible with both **Python 2** and **Python 3**.

Requires **Gurobi**.

### LINK TO THE PAPER ON multiple flying sidekick traveling salesman problem (mFSTSP):

https://papers.ssrn.com/sol3/papers.cfm?abstract_id=3338436


### DESCRIPTION:

This repository provides source codes to run the mFSTSP. The mFSTSP is a variant of the classical TSP, in which one or many UAVs coordinate with a truck to deliver parcels in the minimum possible time. The codes provided here consist of both the integer programming (IP) implementation, and the heuristic algorithm (to solve larger problems). To run this package, it is necessary to have the python modules outlined below. The details on how to run this script (and the options available) is provided below.


### INSTALLING ADDITIONAL PYTHON MODULES (only for Linux. For Windows/Mac, use the command line arguments accordingly):

1) pandas:
```
sudo pip install pandas
```

2) scipy:
```
sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas
```

3) geopy
```
sudo pip install geopy
```

### CONTENTS OF THIS PACKAGE:

1) It contains a main python script called 'main.py' which calls all other python scripts in this package to solve the mFSTSP.

2) It contains a 'Problems' folder, that has 100 subfolders in it. Each subfolder corresponds to a different problem instance.

    i. In each subfolder, you will find a file called 'tbl_locations.csv' which has the lat-lon information of the depot and all the customer nodes.
    
    ii. Each subfolder also contains a file called 'tbl_truck_data_PG.csv' which has the travel time information from each node to all the other nodes.
    
    It is necessary to have these two files in the subfolder corresponding to a problem instance, in order to solve the mFSTSP for that instance.
    
3) The problems folder also contains four CSV files, that have the information on UAV specifications. Depending on the CSV file we choose (between 101, 102, 103, and 104) to solve a problem instance, we select a set of UAVs with particular speed and range. The appropriate CSV file can be chosen through the command line argument, which is described in 'RUNNING THE SCRIPT' section.


### RUNNING THE SCRIPT:

1. Download all files and folders of this package at the same location (inside a folder) in your computer. Name that folder 'mFSTSP'.

2. Go to terminal and and change directory to this folder:
```
cd ~/mFSTSP
```
3. Execute the following command:
```
python main.py <problemName> <vehicleFileID> <cutoffTime> <problemType> <numUAVs> <numTrucks> <requireTruckAtDepot> <requireDriver> <Etype> <ITER>
```

**Description of the command line arguments:**

* problemName: Name of the folder containing the data for a particular problem instance

* vehicleFileID: 101, 102, 103, 104 --> Chooses a particular UAV type depending on the file ID.

* cutoffTime: Gurobi cut-off time (in seconds) of IP model (e.g. 3600). In case of heuristic, it is the Gurobi cut-off time (in seconds) of (P3) model (e.g. 5).

* problemType: 1 (to solve mFSTSP using IP) or 2 (to solve mFSTSP using Heuristic).

* numUAVs: Number of UAVs available in the problem (e.g. 3).

* numTrucks: Number of trucks available in the problem (currently only solvable for 1 truck). Assigning -1 to this parameter ignores its value and considers 1 truck.

* requireTruckAtDepot:  0 (false) or 1 (true) --> If the truck is required to launch/retrieve at the depot.

* requireDriver: 0 (false) or 1 (true). False --> UAVs can launch/land without driver (so driver can serve customer).

* Etype: 1 (NON-LINEAR), 2 (LINEAR), 3 (CONSTANT), 4 (UNLIMITED), 5 (CONSTANT DISTANCE) --> Endurance model being used.

* ITER: Number of iterations the heuristic runs at each LTL. Not applicable when running the IP model, therefore it can be assigned any value when running the IP. (Note that in the heuristic described in the paper, ITER = 1 is considered throughout)

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; **e.g.**

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Solving the mFSTSP optimally:
   ```
   python main.py 20170608T121632668184 101 3600 1 3 -1 1 1 1 -1
   ```

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Solving the mFSTSP via a heuristic:
   ```
   python main.py 20170608T121632668184 101 5 2 3 -1 1 1 1 1
   ```
   
4. You have solved the problem when you get the following type of message on terminal:
```
See 'performance_summary.csv' for statistics.

See 'Problems/20170608T121632668184/tbl_solutions_101_3_Heuristic.csv' for solution summary.
```

### SOLUTION OUTPUT:

Once the code has run, it outputs the following:

1) It adds a row in the 'performance_summary.csv' file. This row has information on quantities such as total run time, objective function value, number of customers assigned to truck, number of customers assigned to UAVs, etc.

2) It also adds an assignment table in the 'tbl_solutions_vehicleFileID_numUAVs_problemType.csv' file, which is inside the subfolder (e.g. 20170608T121632668184) of the problem instance. The assignment table consists of travel and service times of different vehicles, the types of activities that they are performing, as well as their timelines. This is the schedule that will be followed by the truck-UAV system for those set of customers.
