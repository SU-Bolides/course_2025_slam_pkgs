#!/usr/bin/env python3

__author__ = "Nicolas Hammje"
__email__ = "me@nicolashammje.fr"
__status__ = "Tested"
__version__ = "1.0.0"

import rospy
import spidev 
from std_msgs.msg import Float32MultiArray, Int16
from sensor_msgs.msg import Range, Imu
from perception_bolide.msg import ForkSpeed, MultipleRange
import math




class STM32_Parser:
    def __init__(self):
        self.BAUDRATE = 112500
        
        bus=0 
        device=1

        self.stm_pub = rospy.Publisher('stm32_sensors', Float32MultiArray, queue_size=10)
        self.speed_pub = rospy.Publisher('raw_fork_data', ForkSpeed, queue_size=10)
        self.ranges_pub = rospy.Publisher('raw_rear_range_data', MultipleRange, queue_size=10)
        self.imu_pub = rospy.Publisher('raw_imu_data', Imu, queue_size=10)

        self.get_cmd = rospy.Subscriber('stm32_data', Int16, self.get_command)



        self.spi = spidev.SpiDev()
        self.spi.open(bus,device)
        self.spi.mode = 0
        self.spi.max_speed_hz = self.BAUDRATE 


        self.vbat = 0.
        self.yaw = 0.
        self.ir_gauche = 0.
        self.ir_droit = 0.
        self.speed = 0.
        self.distance_US = 0.

        self.tx_buffer = [0] * 8

        self.acc_x = 0.
        self.yaw_rate = 0.

        self.sensor_data = Float32MultiArray() # table for the sensors data
        self.fork_data = ForkSpeed()
        self.imu_data = Imu()

        self.imu_data.angular_velocity_covariance = [0,0,0,0,0,0,0,0,1e-2]
        self.imu_data.linear_acceleration_covariance = [1e-1,0,0,0,0,0,0,0,0]
        self.imu_data.orientation_covariance = [0,0,0,0,0,0,0,0,1e-2]
        self.imu_data.header.frame_id = "base_link"

        self.ir_min_range = 0.06
        self.ir_max_range = 0.3

        self.sensors_init()

    def get_command(self, msg:Float32MultiArray):
        command = []
        cmded_bytes = msg.data#.to_bytes(2, 'big')
        # print("data", msg.data)
        command.append((cmded_bytes >> 8) & 0xFF)
        command.append(cmded_bytes & 0xFF)

        # print("cmd", command)

        crc = self.crc32mpeg2(command)

        command.append((crc >> 24) & 0xFF)
        command.append((crc >> 16) & 0xFF)
        command.append((crc >> 8) & 0xFF)
        command.append((crc) & 0xFF)

        self.tx_buffer = command + [0] * 2
        #This should be not too big.



    # Definitely not the most efficient way of doing this, but we're transmitting 16 bytes so it's OK.
    # Ideally we'd do all this in Cpp, but I don't think there is a spidev equivalent in Cpp, and it's 
    # not like we're being limited by this script. 
    def crc32mpeg2(self,buf, crc=0xffffffff):
        for val in buf:
            crc ^= val << 24
            for _ in range(8):
                crc = crc << 1 if (crc & 0x80000000) == 0 else (crc << 1) ^ 0x104c11db7
        return crc
    

    def sensors_init(self):
        """Initialize the sensors settings and the MultiRange message"""
        self.multi_range_frame = MultipleRange()

        # Rear IR sensors ===============================================================
        self.multi_range_frame.IR_rear_left = Range()
        self.multi_range_frame.IR_rear_left.header.frame_id = "rear_ir_range_frame"
        self.multi_range_frame.IR_rear_left.radiation_type = Range.INFRARED
        self.multi_range_frame.IR_rear_left.min_range = self.ir_min_range
        self.multi_range_frame.IR_rear_left.max_range = self.ir_max_range

        self.multi_range_frame.IR_rear_right = Range()
        self.multi_range_frame.IR_rear_right.header.frame_id = "rear_ir_range_frame"
        self.multi_range_frame.IR_rear_right.radiation_type = Range.INFRARED
        self.multi_range_frame.IR_rear_right.min_range = self.ir_min_range
        self.multi_range_frame.IR_rear_right.max_range = self.ir_max_range
        
        # Rear sonar sensor =============================================================
        # self.multi_range_frame.Sonar_rear = Range()
        # self.multi_range_frame.Sonar_rear.header.frame_id = "rear_sonar_range_frame"
        # self.multi_range_frame.Sonar_rear.radiation_type = Range.ULTRASOUND
        # self.multi_range_frame.Sonar_rear.min_range = self.sonar_min_range
        # self.multi_range_frame.Sonar_rear.max_range = self.sonar_max_range

        

    def receiveSensorData(self):
        self.spi.writebytes(self.tx_buffer)
        data = self.spi.readbytes(20)


        if (not self.crc32mpeg2(data)):
            self.vbat = (data[0] << 8) | data[1] # UNIT??? It's just an ADC, but I think it clips bc it's always 4095 lmao
            self.yaw = ((data[2] << 8) | data[3])/-900. #rad
            self.ir_gauche = (data[4] << 8) | data[5]
            self.ir_droit = (data[6] << 8) | data[7] 
            self.speed = 0.002*((data[8] << 8) | data[9]) #ms^-1 # We have to add a factor 2 for some reason, don't ask me why though. 
            self.distance_US = 0.01*((data[10] << 8) | data[11]) #m
            self.acc_x = 0.01*int.from_bytes([data[12],data[13]], byteorder='big', signed=True) #ms^-2
            self.yaw_rate = int.from_bytes([data[14],data[15]], byteorder='big', signed=True)/900. #rads^-1

            self.sensor_data.data = [self.yaw, self.speed, self.ir_gauche, self.ir_droit, self.distance_US, self.acc_x, self.yaw_rate]

            self.fork_data.speed = self.speed
            self.fork_data.header.stamp = rospy.Time.now()



            ##########################  RANGES  #############################

            # Left ====================================
            IR_rear_left = self.ir_gauche/1000.0 # conversion mV --> V
            if(IR_rear_left):
                IR_rear_left = 15.38/IR_rear_left - 0.42 # conversion V --> cm
            else:
                IR_rear_left = self.ir_max_range*100. #because we convert in m later
            # using component datasheet
            # ... and with a bit of experimentation and test to adapt the value
            IR_rear_left /= 100 # conversion cm --> m
            IR_rear_left = min(max(IR_rear_left,0), self.ir_max_range)
            

            # Right ===================================
            IR_rear_right = self.ir_droit/1000. # conversion mV --> V
            if(IR_rear_right):
                IR_rear_right = 15.38/IR_rear_right - 0.42 # using component datasheet
            else:
                IR_rear_right = self.ir_max_range*100. #because we convert in m later
            # using component datasheet
            # ... and with a bit of experimentation and test to adapt the value
            IR_rear_right = IR_rear_right/100. # conversion cm --> m
            IR_rear_right = min(max(IR_rear_right,0), self.ir_max_range)

            # self.multi_range_frame.Sonar_rear.range = Sonar_rear
            # self.multi_range_frame.Sonar_rear.header.stamp = rospy.Time.now()
            self.multi_range_frame.IR_rear_left.range = IR_rear_left
            self.multi_range_frame.IR_rear_left.header.stamp = rospy.Time.now()
            self.multi_range_frame.IR_rear_right.range = IR_rear_right
            self.multi_range_frame.IR_rear_right.header.stamp = rospy.Time.now()

            #########################################################

            self.imu_data.orientation.z = math.sin(self.yaw*0.5)
            self.imu_data.orientation.w = math.cos(self.yaw*0.5)

            self.imu_data.angular_velocity.z = self.yaw_rate
            # self.imu_data.linear_acceleration.x = self.acc_x + 0.043

            stamp = rospy.Time.now()
            self.fork_data.header.stamp = stamp
            self.imu_data.header.stamp = stamp
            self.stm_pub.publish(self.sensor_data)



        if (not rospy.is_shutdown()):
            self.speed_pub.publish(self.fork_data)
            self.ranges_pub.publish(self.multi_range_frame)
            self.imu_pub.publish(self.imu_data)

        
if __name__ == '__main__':

    Parser = STM32_Parser()    
    rospy.init_node('stm32_publisher')
    rate = rospy.Rate(200)
    while(not rospy.is_shutdown()):
        Parser.receiveSensorData()
        rate.sleep()