# DDM: Database-Driven Collision Resolution
This repository includes the implementation of the collision resolution part of the
[DDM algorithm](https://arxiv.org/pdf/1904.02598.pdf). 

Path diversification not included for now. 

## Prerequisites
C++17, 
[boost library](https://www.boost.org/), 
[libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) for single robot planning.

## How to use
The implementation is in Visual Studio 2015.

Compile and run `main.cpp`. 

If you encounter assertion `ddm_database[1].size() == 36` failure, please make sure the database folder is at the same directory as the program executable.

### Other info
1. [DDM video](https://youtu.be/0MUGrg5CphM). 
1. Main difference from the original DDM publication: the "corner case" (Fig. 3(e) in the paper) is now resolved using a 3x3 database with one obstacle. 