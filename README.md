# Repository of the ROS Course Project 2025
Work on progress...
## Changes and Changes
The main goal of this year is to pass the project from ROS to ROS2 (jazzy). By doing this, we also want to help clarify packages and how nodes work together. If we have time after the course (because clearly, we *don't* have the time before it), we will try to add a camera on the car.

The Repository is divided in three parts

## Workspace
Normally if we're brave enough, we will try to add a README to all packages, so if you want to have a better explanation of a point try in its folder.
This folder contains the working workspace of ROS2. You find there : 
- The build files made by colcon.
- A Python3 virtual environment (venv) for external library needed in python (since Ubuntu doesn't allow pip installation on global environment), you don't need to touch anything there normally?
- Our source folder (src) where all packages are. It's your mainly source of hope and despair so you need to understand it clearly. Hopefully, ROS2 is more rigorous than ROS in the construction of packages so for every packages, the structure is the same and it is easier to understand nodes.