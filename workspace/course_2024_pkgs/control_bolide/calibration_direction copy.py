#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from dynamixel_sdk import *
import time 
import math
#paramètres de départ, avec des butées très proche du centre


# Control table address
ADDR_PRO_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_PRO_LED_RED            = 25
ADDR_PRO_GOAL_POSITION      = 30
ADDR_PRO_PRESENT_POSITION   = 36

# Data Byte Length
LEN_PRO_LED_RED             = 1
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 
BAUDRATE                    = 115200            
DEVICENAME                  = '/dev/ttyU2D2'    # Symlink it in the udev to ttyU2D2 
                          

portHandler = PortHandler(DEVICENAME)

print("ad")

packetHandler = PacketHandler(PROTOCOL_VERSION)

#Open port to U2D2 
if portHandler.openPort():
    print("[INFO] Succeeded to open the port")
else:
    print("[ERROR] Failed to open the port")
    quit()

#Set the baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("[INFO] Succeeded to change the baudrate")
else:
    print("[ERROR] Failed to change the baudrate")
    quit()
#Torque off 
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 0)

#LED on
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_LED_RED, 1)

# #Torque on
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1)





direction = 1 #1 pour angle_pwm_min a gauche, -1 pour angle_pwm_min à droite
angle_pwm_min = 50 #min
angle_pwm_max = 237 #max
angle_pwm_centre= 150
angle_degre_max = +14.8 #vers la gauche
angle_degre=0

# pos, result, error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)

# print(pos)

def degrees2pos(degrees):
    return int((degrees - 30.) * (1023./300.))


def pos2psi(pos):
    # Psi is the steering angle (in radians) 
    # Theta is the motor angle (in radians) 
    # Pos is the motor angle (in DXL units)
    theta_rad = (pos * (5.24/1023.)) + 0.524
    A = math.cos(theta_rad) * 15 - 4.2
    psi_rad = math.asin(A/25.)
    return(psi_rad)


def pos2degrees(pos):
    return (pos * (300./1023.)) + 30


def set_direction_degre(angle_degre) :
    psi = math.radians(angle_degre)
    A = 25 * math.sin(psi) + 4.2

    print(A, "mm")

    theta = math.acos(A/15.)

    print(theta)

    pos = degrees2pos(math.degrees(theta))
    packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, pos)




    

# dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, int(angle_pwm_centre))
print("réglage des butées, Q pour quitter")
print("valeur numérique pour tester un angle de direction")
print("I pour inverser droite et gauche")
print("g pour diminuer la butée gauche et G pour l'augmenter")
print("d pour diminuer la butée droite et D pour l'augmenter")
while True :
    pos, result, error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    print(math.degrees(pos2psi(pos)))
    # a = input("g, G, d, D ?")
    # angle_degre=int(a)
    # set_direction_degre(angle_degre)
    time.sleep(0.5)


    
print("nouvelles valeurs")
print("direction : " + str(direction))
print("angle_pwm_min : " + str(angle_pwm_min))
print("angle_pwm_max : " + str(angle_pwm_max))
print("angle_pwm_centre : " + str(angle_pwm_centre))
