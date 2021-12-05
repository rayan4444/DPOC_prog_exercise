# DPOC_prog_exercise

This repository contains the files in the context of the Dynamic Programming course project. The exercise involves a drone package delivery problem. The drone must first pickup and then deliver the package, whilst avoiding hazards such as trees or the angry residents attempting to shoot it down. The delivery problem is set up and solved using 3 dynamic programming approaches, namely value iteration, policy iteration and linear programming. 

The code is run and checked with the Check_Matrix function as follows: 

1. First run the main file in order to compute the self coded P and G matrices for the example matrices. In order to get the example world within 'main'set generateRandomWorld = false. Given the example_G and example_P data files within the same folder path, subsequently running Check_Matrix will determine if the two correspond. Retuned 0 means they do not, returned 1 means they do. 
