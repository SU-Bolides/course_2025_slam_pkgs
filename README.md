# Repository of the ROS Course Project 2025
Work in progress...
## Changes and Changes
The main goal of this year is to pass the project from ROS to ROS2 (jazzy). By doing this, we also want to help clarify packages and how nodes work together. If we have time after the course (because clearly, we *don't* have the time before it), we will try to add a camera on the car.

The Repository is divided in three parts

## Workspace
Normally if we're brave enough, we will try to add a README to all packages, so if you want to have a better explanation of a point try in its folder.
This folder contains the working workspace of ROS2. You find there : 
- The build files made by colcon.
- A Python3 virtual environment (venv) for external library needed in python (since Ubuntu doesn't allow pip installation on global environment), you don't need to touch anything there normally?
- Our source folder (src) where all packages are. It's your mainly source of hope and despair so you need to understand it clearly. Hopefully, ROS2 is more rigorous than ROS in the construction of packages so for every packages, the structure is the same and it is easier to understand nodes.

## General Information
If you want to transform a package from ROS to ROS2 or to add your own package, please follow these instructions correctly to avoid losing time and to keep a clean and clear environment. All these informations are mainly inspired by the official [tutorials](https://docs.ros.org/en/jazzy/Tutorials.html) of ROS2 (here jazzy distribution), that you followed at the beginning of the class.
### Create a Package
To create a package go to the [src](./workspace/src/) folder of your workspace on a terminal and use the command line :
```shell
create pkg —build-type ament_python —license Apache-2.0 package_name
```
After the creation done you will see your folder and all the basic files needed.
#### Package from external source
If you want to use a package already made by someone for a sensor (e.g: the lidar), then just proceed to the instructions given by the creator (generally on the README file of the github page) and make sure to be at the good place.
#### Node
In your node be as clear as possible. We're not in the 80s and we got enough technology to help us coding that you can be clear and precise. Don't name your value like 'a' or 'br', it is not easy to understand their meaning so name them like 'acceleration' or 'baudrate'. Like this we can understand in one read what represent the value and we don't need to add comments everywhere. It is also recommended to describe your file, class, function, ..., put your name and surname at the beginning to know who contact if we need it. By describing this element we can, again, know easily what does the module without reading 3 hours by files.
##### Publisher/Subscriber
For the topic name use something understandable (like '/lidar_data') or some basic naming in ROS2 (like '/cmd_vel'). Put a '/' at the beginning of the topic name.
##### Messages/Services
For messages and services that you create, put them all in one and only one package. Here this package is [bolide_interfaces](./workspace/src/bolide_interfaces/).
##### Launch files
If you want to use launch files create in your package folder a 'launch' folder where you will put your launch codes. Name your file like 'name_of_the_launch.launch.py'. If a package have a launch file, never forget to add in its 'setup.py' file this :
```python
data_file=[
    ...
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'launch.[pxy][yma]'))),
]
```
##### Setup and Package file
In your 'setup.py' file you need to add all your nodes name and source like this :
```python
entry_points={
        'console_scripts': [
            'node_name = pkg_name.node_file_name:main',
        ],
    },
```
In your 'package.xml' you need to add all the dependencies of your package like this (e.g):
```xml
<depend>rclpy</depend>
<depend>another_pkg</depend>
```
If a package is dependant of another package in your workspace you will need to build firstly the dependence.
##### Python package
In your code you will surely use some external Python packages like 'spidev'