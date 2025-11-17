# Robotics, Lab assignment 1

P2, 2025, joao.silva.sequeira@tecnico.ulisboa.pt  
Due on: 12-12-2025, 23:59:59

## Introduction
The objective of the lab is to show how a functionality nowadays present is many modern cars can
be approached using the ideas discussed in the theory classes.

Consider a car moving (i.e., being driven by a human) in a highway, where each lane is clearly
delimited by white lines and, in general, unobstructed.

Ideally, the car must stay in the middle of a lane, except when overtaking other cars/obstacles. 

For the purpose of this lab assume that no other cars or obstacles are present in the highway and the car is to stay inside a given lane.

The goal of a LTA system is to, automatically, steer the vehicle, such that it stays inside the assigned lane, using information from adequate sensors installed onboard the car.

There is an extensive online documentation on LTA systems developed by several manufacturers,
and there maybe some differences among their respective behaviours. 

YouTube videos, may also be interesting to clarify multiple aspects.

In this lab a LTA system is to be simulated, using the ideas discussed in the theory classes.

Either Matlab or Python can be used to solve the computational tasks.

## Tasks

### Task 1
Develop a joystick-like function, i.e., using 2 keys in the computer keyboard (or a gaming
pad) to steer right/left a simulated car. There are multiple Python libraries available online that can be used.

### Task 2a
Develop a model for the car using the principles discussed in the theory classes, i.e. the
geometric models. A kinematic model is enough for the purpose of the lab, though obtaining a
realistic behaviour requires, in general, a dynamics model.

### Task 2b
Develop models for the sensors onboard the car. You are free to choose the sensors but
they must have a connection with real sensors.

### Task 3
Show that, with the joystick-controlled steering, a simulated car can indeed stay within the
lines of a road lane, i.e., develop a simulation environment with a road lane and show that just by using the joystick you, i.e., a person in charge of driving the car, can keep the car within the lane limits.

### Task 4
Using the simulation environment developed in task 3, plot the detection of the lane’s left
and right white lines as a function of time. The idea is to have a visual notion of the quality of the line detection system.

### Task 5
Develop a controller for LTA that can be integrated in the architecture developed in task 3.
Carefully, explain how the LTA developed works. Show the LTA effectiveness (use, for example,
empiric strategies).

### Task 6
Using mathematical arguments demonstrate the effectiveness of the proposed LTA.

## Deliverables
• Report detailing the techniques used and results obtained. The role of each author must be
explicitly stated in the report.  
• All the software produced, including comprehensible instructions to allow anyone to install
and run it.

## Suggested schedule
Tasks 1, 2a, 2b - 17 November 2025  
Task 3 - 21 November 2025  
Task 4 - 30 November 2025  
Task 5 - 5 December 2025  
Task 6 - 12 December 2025
