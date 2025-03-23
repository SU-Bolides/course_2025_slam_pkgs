import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Range
from std_msgs.msg import Float32MultiArray
from bolide_interfaces.msg import MultipleRange

class RearSensorPublisher(Node):
    def __init__(self):
        super().__init__('rear_ranges_publisher')

        # SUBSCRIBER 
        self.subscription = self.create_subscription(Float32MultiArray, '/stm32_sensors', self.callback, 10)
        # PUBLISHER
        self.pub = self.create_publisher(MultipleRange, '/raw_rear_range_data', 10)

        #defining ranges of the IR
        self.ir_min_range = 0.06
        self.ir_max_range= 0.3

        # # defining ranges of the US
        # self.sonar_min_range = 0.07
        # self.sonar_max_range = 0.5

        # init the static data of the sensors
        self.sensors_init()

    def sensors_init(self):
        """Initialize the sensors settings and the MultiRange message"""
        self.multiRangeFrame = MultipleRange()

        # Rear IR sensors ===============================================================
        self.multiRangeFrame.ir_rear_left = Range()
        self.multiRangeFrame.ir_rear_left.header.frame_id = "rear_ir_range_frame"
        self.multiRangeFrame.ir_rear_left.radiation_type = Range.INFRARED
        self.multiRangeFrame.ir_rear_left.min_range = self.ir_min_range
        self.multiRangeFrame.ir_rear_left.max_range = self.ir_max_range

        self.multiRangeFrame.ir_rear_right = Range()
        self.multiRangeFrame.ir_rear_right.header.frame_id = "rear_ir_range_frame"
        self.multiRangeFrame.ir_rear_right.radiation_type = Range.INFRARED
        self.multiRangeFrame.ir_rear_right.min_range = self.ir_min_range
        self.multiRangeFrame.ir_rear_right.max_range = self.ir_max_range

        # Rear sonar sensor =============================================================
        # self.multiRangeFrame.Sonar_rear = Range()
        # self.multiRangeFrame.Sonar_rear.header.frame_id = "rear_sonar_range_frame"
        # self.multiRangeFrame.Sonar_rear.radiation_type = Range.ULTRASOUND
        # self.multiRangeFrame.Sonar_rear.min_range = self.sonar_min_range
        # self.multiRangeFrame.Sonar_rear.max_range = self.sonar_max_range

    def callback(self, data:Float32MultiArray):
        """ Callback function called when a message is received on the subscribed topic"""
 
        # retrieving Rear data (2xIR + sonar) from the STM32_sensors msg
        # Sonar_rear = data.data[3] #sonar
        IR_rear_left = data.data[2] #IR left
        IR_rear_right = data.data[3] #IR right

        # Process sonar data
        # Sonar_rear = Sonar_rear/100 # passage de cm à m

        # Process IR's data
        # ========== conversion mV --> m ==========
        # Left ====================================
        IR_rear_left = IR_rear_left/1000 # conversion mV --> V
        if(IR_rear_left == 0):
            IR_rear_left = self.ir_max_range*100 #because we convert in m later
        else :
            IR_rear_left = 15.38/IR_rear_left - 0.42 # conversion V --> cm
        # using component datasheet
        # ... and with a bit of experimentation and test to adapt the value
        IR_rear_left = IR_rear_left/100 # conversion cm --> m
        if(IR_rear_left < self.ir_min_range): # on met à 0 en dessous d'un certain seuil, cf doc
            IR_rear_left = 0
        elif(IR_rear_left > self.ir_max_range):
            IR_rear_left = self.ir_max_range

        # Right ===================================
        IR_rear_right = IR_rear_right/1000 # conversion mV --> V
        if(IR_rear_right == 0):
            IR_rear_right = self.ir_max_range*100 #because we convert in m later
        else :
            IR_rear_right = 15.38/IR_rear_right - 0.42 # using component datasheet
        # using component datasheet
        # ... and with a bit of experimentation and test to adapt the value
        IR_rear_right = IR_rear_right/100 # conversion cm --> m
        if(IR_rear_right < self.ir_min_range): # on met à 0 en dessous d'un certain seuil, cf doc
            IR_rear_right = 0
        elif(IR_rear_right > self.ir_max_range):
            IR_rear_right = self.ir_max_range

        # self.multiRangeFrame.Sonar_rear.range = Sonar_rear
        # self.multiRangeFrame.Sonar_rear.header.stamp = rospy.Time.now()
        self.multiRangeFrame.ir_rear_left.range = IR_rear_left
        self.multiRangeFrame.ir_rear_left.header.stamp = self.get_clock().now().to_msg()
        self.multiRangeFrame.ir_rear_right.range = IR_rear_right
        self.multiRangeFrame.ir_rear_right.header.stamp = self.get_clock().now().to_msg()

        self.pub.publish(self.multiRangeFrame)

def main(args = None):
    rclpy.init(args=args)

    try:
        Rear_data = RearSensorPublisher()
    except KeyboardInterrupt: #rospy.ROSInterruptException normalement mais je dois trouver l'équivalence
        exit(0)

