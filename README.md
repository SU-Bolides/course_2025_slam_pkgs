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

### Launching the car (for now)
When the car is on, connect with ssh (you have to know its ip address before), or branch it to a display, then on the terminal check the connection of all USB devices (Lidar and Dynamixel) with :
```shell
ls /dev/ttyUSB*
```
You will see /dev/ttyUSB0 and /dev/ttyUSB1. Disconnect the Lidar from the Rpi5 (the top one), then retry the command and see which one doesn't appear now. Check if the one that doesn't appear now is the one put in the [launchfile](./workspace/src/perception_bolide/launch/perception.launch.py) at the perception_bolide package. If it isn't correctly paired change it and go to the [ackermann_controller.py](/workspace/src/control_bolide/control_bolide/ackermann_controller.py) file to change also the device of the Dynamixel. After doing that type this command in the terminal:
```shell
sudo chmod 777 /dev/ttyUSB*
```
Now go to the workspace 
```shell
cd course_2025_slam_pkgs/workspace/
```
Source your workspace and you can press on the bottom button near the screen display on the car. Then source the virtual environment and launch the perception process:
```shell
ros2 launch perception_bolide perception.launch.py
```
On another terminal now, source again your workspace and the virtual environment and run ackermann_controller node:
```shell
ros2 run control_bolide ackermann_controller
```
And to teleoperate with the keyboard open a last terminal, source your workspace and run:
```shell
ros2 run planning_bolide teleop_node
```
Normally, you're good to move the car with the arrows keys of your keyboard. 

### Bolide Interfaces
Bolide Interfaces [package](./workspace/src/bolide_interfaces/) is where you will find every Messages and Services that we created for the car. For now, we mainly use messages between topics, the messages are :
- **ForkSpeed** composed of a header and a float32 value corresponding to the speed measures by the fork
- **MultipleRange** composed of 3 Ranges (from std_msgs). One for the Rear left Infrared sensors, one for the right and the last for the Sonar (not used for now)
- **SpeedDirection** composed of two float64 values, one for the speed of the robot and the other for the direction. Both are between -1 and 1.
To access in a python file, you need to import the package (e.g from bolide_interfaces/msg import SpeedDirection)

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
#### Setup and Package file
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
#### Python package
In your code you will surely use some external Python packages like 'spidev', but recently Ubuntu rules about global environment changed and we can't just install with pip the package. To be able to use external package we use a virtual environment. The virtual environment is in the [/venv](./workspace/venv/) folder. To activate it use this command line in the workspace:
```shell
source /venv/bin/activate
```
All command usually working in the terminal still work here. If you want to install some packages use 'pip install package_name'. With this you can use any package in python for your ROS2 node. You will need to go in the 'setup.cfg' file of your package and add this :
```pkg
[build_scripts]
executable = /usr/bin/env python3
```
#### At the end
At the end our workspace (without build files) need to look like this:
- workspace/
  - bolides_interfaces/
  - pkg1/
  - pkg2/
  - external_pkg1/
  - external_pkg2/
  - ...

## Generals Problems
### Battery
When we're using the propulsion motors, the voltage doesn't keep up and it provokes the shutdown of the RPi5. We think that it is caused by the oldness of the batteries. To fix it, we can buy new battery, maybe with a higher voltage capacity or we could branch two batteries in parallel to give more current. Another idea is to use a second battery only for the RPi5 and the main battery for the motors, to safely use them without crashing the computer.


---

### Tutorial

Follow these steps to control the car from your PC:

1. **Connect to the Same Network (Or any other VMs just make sure that the 4th step is made)**  
   - Connect both your Raspberry Pi 5 (Rpi5) and your PC (VMware) to the same hotspot (e.g., your 4G/5G or Wi-Fi).  
   - For now, the default 4G connection on the Rpi5 is: `iPhone de Babou (2)`. A more generic Wi-Fi will be added later.

2. **Test the Connection**  
   - On your Rpi5, run the following command to get its IP address:  
     ```shell
     hostname -I
     ```
     Note the first IP address (`<IP_ROBOT>`).  
   - On your PC, test the connection by pinging the Rpi5:  
     ```shell
     ping <IP_ROBOT>
     ```
     You should see output similar to this:  
     ```
     PING 172.20.10.9 (172.20.10.9) 56(84) bytes of data.
     64 bytes from 172.20.10.9: icmp_seq=1 ttl=64 time=1025 ms
     64 bytes from 172.20.10.9: icmp_seq=2 ttl=64 time=7.18 ms
     64 bytes from 172.20.10.9: icmp_seq=3 ttl=64 time=146 ms
     64 bytes from 172.20.10.9: icmp_seq=4 ttl=64 time=10.8 ms
     64 bytes from 172.20.10.9: icmp_seq=5 ttl=64 time=10.8 ms
     ```
   - Repeat the same steps in reverse:  
     On your PC, run `hostname -I` and ping the PC's IP address from the Rpi5.

3. **Set the ROS Domain ID**  
   - On your Rpi5, check the ROS Domain ID:  
     ```shell
     echo $ROS_DOMAIN_ID
     ```
     By default, it should be `10`.  
   - On your PC (VMware), set the same ROS Domain ID:  
     ```shell
     export ROS_DOMAIN_ID=10
     ```

4. **Launch the Teleoperation Node**  
   - On your PC, launch the teleoperation node from the `planning_bolide` package:  
     ```shell
     ros2 run planning_bolide teleop_node
     ```

---

### Troubleshooting

If you encounter any issues, please double-check the steps above or ask for assistance. Some key steps might have been overlooked. 

