# The Multiple Flying Sidekick Traveling Salesman Problem (mFSTSP)

This repository provides a collection of mFSTSP test problems, as well as the source code to solve mFSTSP instances. The mFSTSP is a variant of the classical TSP, in which one or more UAVs coordinate with a truck to deliver parcels in the minimum possible time. The code provided here consists of both the mixed integer linear programming (MILP) implementation and a heuristic to solve larger problems. 

The repository accompanies the following paper, which is currently under a second round of reviews:
> C. Murray and R. Raj, The Multiple Flying Sidekicks Traveling Salesman Problem: Parcel Delivery with Multiple Drones (July 27, 2019). Available at SSRN: https://ssrn.com/abstract=3338436 or http://dx.doi.org/10.2139/ssrn.3338436

The paper provides details on the mFSTSP definition, the corresponding MILP formulation, and the heuristic.


## mFSTSP Repository Contents
This repository contains the following materials:

1. A collection of **test problems (and solutions)** that were used in the analysis described in the [mFSTSP paper](https://ssrn.com/abstract=3338436).

   1. [`InstanceInfo.csv`](InstanceInfo.csv) contains a summary of all 100 "base" test problems, including the number of customers, geographic region (Buffalo, NY or Seattle, WA), and the number of customers within 5 or 10 miles of the depot.  This file may be useful if you are looking for problems with certain properties (e.g., 50-customer problems, or problems in Seattle). 
   
   2. [`performance_summary_archive.csv`](performance_summary_archive.csv) provides information about solutions generated for each test problem.  This file contains the following columns:
      - `problemName` - The name of the problem instance (written in the form of a timestamp).  This is a subdirectory name within the [`Problems`](Problems) directory.
      - `vehicleFileID` - Indicates the speed and range of the UAVs.  See the note below, which describes the definitions of `101`, `102`, `103`, and `104`.
      - `cutoffTime` - The maximum allowable runtime, in [seconds].  If the problem was solved via heuristic, this time represents the maximum runtime of Phase 3.
      - `problemType` - Indicates if the problem was solved via MILP (`1`) or heuristic (`2`).
      - `problemTypeString` - A text string describing the solution approach (`mFSTSP IP` or `mFSTSP Heuristic`).
      - `numUAVs` - # of UAVs available (`1`, `2`, `3`, or `4`).
      - `numTrucks` - This can be ignored; it will always have a value of `-1`.  However, in each problem there is actually exactly 1 truck.
      - `requireTruckAtDepot` - A boolean value to indicate whether the truck is required to be at the depot when UAVs are launched from the depot.  This problem "variant" is described in Section 3 of the [mFSTSP paper](https://ssrn.com/abstract=3338436).
      - `requireDriver` - A boolean value to indicate if the driver is required to be present when UAVs are launched/retrieved by the truck at customer locations.  This problem "variant" is described in Section 3 of the [mFSTSP paper](https://ssrn.com/abstract=3338436).
      - `Etype` - Indicates the endurance model that was employed.  Options include: `1` (nonlinear), `2` (linear), `3` (fixed/constant time), `4` (unlimited), and `5` (fixed/constant distance).  Details on these models are found in Section 4 of the [mFSTSP paper](https://ssrn.com/abstract=3338436). 
      - `ITER` - Indicates the number of iterations to be run for each value of "LTL".  If `problemType == 1` (MILP), `ITER` will be -1 (not applicable).  Otherwise, `ITER` will be `1` for the heuristic.  NOTE: This feature is not described/implemented in the [mFSTSP paper](https://ssrn.com/abstract=3338436).
      - `runString` - This contains the command-line arguments used to solve the problem.  It is included here to allow copy/pasting.
      - `numCustomers` - The total number of customers.
      - `timestamp` - The time at which the problem was solved.
      - `ofv` - The objective function value, as obtained by the given solution approach.
      - `bestBound` - If `problemType == 1` (MILP), this is the best bound provided by Gurobi.  Otherwise, `bestBound = -1` for the heuristic (as no bounds are available).
      - `totalTime` - The total runtime of the MILP or heuristic.
      - `isOptimal` - A boolean value to indicate if the solution was provably optimal.  This only applies if `problemType == 1` (MILP); there is no proof of optimality for the heuristic.
      - `numUAVcust` - The number of customers assigned to the UAV(s).
      - `numTruckCust` - The number of customers assigned to the truck.
      - `waitingTruck` - The amount of time, in [seconds], that the truck spends waiting on UAVs.
      - `waitingUAV` - The amount of time, in [seconds], that the UAVs spend waiting on the truck.
      
      **NOTE:** [`performance_summary.csv`](performance_summary.csv) is an empty/placeholder file.  It will be populated only if/when you run the solver code.  This file currently exists solely to initialize the column headings.
       
   3. The `Problems` directory contains 100 sub-directories (one for each "base" problem).
        - In each subdirectory, you will find a file called `tbl_locations.csv` which has the lat/lon information of the depot and all the customer nodes.
        - Each subdirectory also contains a file called `tbl_truck_data_PG.csv` which has the travel time information from each node to all the other nodes.
    It is necessary to have these two files in the subfolder corresponding to a problem instance, in order to solve the mFSTSP for that instance.

   4. The `Problems` directory also contains four CSV files, that have the information on UAV specifications. Depending on the CSV file we choose (between 101, 102, 103, and 104) to solve a problem instance, we select a set of UAVs with particular speed and range. The appropriate CSV file can be chosen through the command line argument, which is described in **RUNNING THE SCRIPT** section.
        - 101: High speed, low range
        - 102: High speed, high range
        - 103: Low speed, low range
        - 104: Low speed, high range

3. A main python script called 'main.py' which calls all other python scripts in this package to solve the mFSTSP.



## Installation and Setup


### Compatability

- The mFSTSP source code is compatible with both **Python 2** and **Python 3**.
- It has been tested on **Windows**, **Linux**, and **Mac**.

### Prerequisites
- Both the MILP and the hueristic require [Gurobi](http://gurobi.com).


#### Additional Required Python Modules
To run this package, it is necessary to have the python modules outlined below. 
The following terminal commands are applicable to Linux and Mac only.  Windows users will need to install these packages in a different manner.  **HOW???**

1. pandas:
   ```
   pip install pandas
   ```
   *If you receive errors related to "access denied", try running `sudo pip install pandas`.*
   
2. geopy:
   ```
   sudo pip install geopy
   ```
   *If you receive errors related to "access denied", try running `sudo pip install geopy`.*

3. scipy:
   ```
   sudo apt-get install python-numpy python-scipy python-matplotlib ipython ipython-notebook python-pandas
   ```












### RUNNING THE SCRIPT:
The details on how to run this script (and the options available) is provided below.

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

* **problemName**: Name of the folder containing the data for a particular problem instance

* **vehicleFileID**: 101, 102, 103, 104 --> Chooses a particular UAV type depending on the file ID.

* **cutoffTime**: Gurobi cut-off time (in seconds) of IP model (e.g. 3600). In case of heuristic, it is the Gurobi cut-off time (in seconds) of (P3) model (e.g. 5).

* **problemType**: 1 (to solve mFSTSP using IP) or 2 (to solve mFSTSP using Heuristic).

* **numUAVs**: Number of UAVs available in the problem (e.g. 3).

* **numTrucks**: Number of trucks available in the problem (currently only solvable for 1 truck). Assigning -1 to this parameter ignores its value and considers 1 truck.

* **requireTruckAtDepot**:  0 (false) or 1 (true) --> If the truck is required to launch/retrieve at the depot.

* **requireDriver**: 0 (false) or 1 (true). False --> UAVs can launch/land without driver (so driver can serve customer).

* **Etype**: 1 (NON-LINEAR), 2 (LINEAR), 3 (CONSTANT), 4 (UNLIMITED), 5 (CONSTANT DISTANCE) --> Endurance model being used.

* **ITER**: Number of iterations the heuristic runs at each LTL. Not applicable when running the IP model, therefore it can be assigned any value when running the IP. (Note that in the heuristic described in the paper, ITER = 1 is considered throughout)

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


### ARCHIVED PROBLEM SOLUTIONS:

The package also contains the solution of the problem instances, which is used in the analysis section of the paper. A file called 'performance_summary_archive.csv' contains the summary of these already-solved problem instances (with different parameter settings). Specifically, it has the solution of following instances:

1. Each 8-customer problem instance (20, total) is run using four different settings of vehicleFileID, and {1,2,3,4} number of UAVs, using both the IP and the Heuristic, resulting in a total of 640 problem solutions.
    
2. Each of the 10-, 25-, 50-, and 100-customer problem instances (80, total) is run using four different settings of vehicleFileID, and {1,2,3,4} number of UAVs, using just the Heuristic, resulting in a total of 1280 problem solutions.
    
The complete parameter (e.g. cutoffTime, ITER, Etype, etc.) specifications for each problem solution can be found in 'performance_summary_archive.csv'. The assignment table corresponding to each of these solutions also exists in the 'tbl_solutions_vehicleFileID_numUAVs_problemType.csv' file, which is inside the subfolder (e.g. 20170608T121632668184) of the problem instance. When a problem is ran again using the same settings (e.g. same vehicleFileID, numUAVs and problemType), it just appends another assignment table in the same csv file.


&nbsp;

### Contact Info:

For any queries, please send an email to r28@buffalo.edu.
