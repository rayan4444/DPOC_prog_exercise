# DPOC_prog_exercise

This repository contains the files in the context of the Dynamic Programming course project. The exercise involves a drone package delivery problem. The drone must first pickup and then deliver the package, whilst avoiding hazards such as trees or the angry residents attempting to shoot it down. The delivery problem is set up and solved using 3 dynamic programming approaches, namely value iteration, policy iteration and linear programming. 

In order to run the code:

First run the main file in order to compute the self coded P and G matrices for the example matrices. In order to get the example world within 'main'set generateRandomWorld = false. Given the example_G and example_P data files within the same folder path, subsequently running Check_Matrix will determine if the two correspond. Retuned 0 means they do not, returned 1 means they do. 

The resulting plots for a randomly generated world are the following: 



![world](https://user-images.githubusercontent.com/47719524/144756889-82de66c2-3d17-4dcb-9a12-a27ae664e82e.PNG)
![value](https://user-images.githubusercontent.com/47719524/144756890-55d34be8-a183-413d-864f-6024a488c48e.png)
![policy](https://user-images.githubusercontent.com/47719524/144756896-3659d752-7a9d-40e5-b7e9-83e7a7527a87.png)
![linear](https://user-images.githubusercontent.com/47719524/144756900-1a3a36cb-d69d-4718-82d9-09c0fa604d0e.png)
