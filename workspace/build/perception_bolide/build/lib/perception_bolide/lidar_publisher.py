import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rplidar import RPLidar, RPLidarException
from sensor_msgs.msg import LaserScan
import time
from numpy import pi

class LidarPublisher(Node):
    def __init__(self):
        super().__init__("lidar_publisher")

        self.publish_rate = 12 # in Hz
        self.min_range = 0.2 # in m
        self.max_range = 12 # in m
        self.scan_time = 1/self.publish_rate # in s
        self.time_increment = self.scan_time/360 # in s (because there are 360 points)

        # Connect to the RPLidar
        self.lidar = RPLidar("/dev/ttyUSB1", baudrate = 256000)
        self.lidar.connect()

        try:
            # Log the info from the lidar
            self.get_logger().info(f"[INFO] -- Lidar connected {self.lidar.get_info()}")
        except Exception as e:
            self.get_logger().error(f"[ERROR] -- {e}\nerror, can't connect to Lidar")
            exit(1)

        # Start the lidar motor
        self.get_logger().info(f"[INFO] -- Starting Lidar")
        self.lidar.start_motor()
        time.sleep(1) # wait for the motor to star

        # Initialize the publisher node
        self.pub = self.create_publisher(LaserScan, '/raw_lidar_data', 10)

        # set all the data that are fixed in LidarScan
        self.lidar_data = LaserScan()
        self.lidar_data.header.frame_id = "lidar_frame"
        self.lidar_data.angle_min = 0 # in radians
        self.lidar_data.angle_max = 359*pi/180 # in radians
        self.lidar_data.angle_increment = pi / 180 # in radians (should be 1 degree)
        self.lidar_data.scan_time = self.scan_time # in seconds
        self.lidar_data.time_increment = self.time_increment # in seconds
        self.lidar_data.range_min = self.min_range # in meters
        self.lidar_data.range_max = self.max_range # in meters

        # We use a thread because when using rospy.Rate in the same method where we collecting the data, it creates rates issues
        # Create and start a separate thread for collecting lidar data
        self.collect_thread = threading.Thread(target=self.collect_data)
        self.collect_thread.start()

        # Create an Event object for signaling the thread to stop
        self.stop_thread = threading.Event()

        # Create a lock for thread-safe access to self.lidar_data
        self.data_lock = threading.Lock()

        # Start publishing lidar data
        time.sleep(1) # wait for the first data to be collected
        self.get_logger().info(f"[INFO] -- Start pulishing on topic /raw_lidar_data")
        self.start_publish()

    def collect_data(self):
        # Continuously fill the array with lidar scan data
        # Create an array of 360 zeros
        ### ============= WARNING ==============
        # Here we initialize the array_lidar once so when no data are received at time t it will keep the data at time t-1
        # But we should initialize every time we do a new scan (first line commented in the for loop below)
        # This config create a cloud of point behind the robot (see with lidar_vizu or rviz)
        # We do this for now to avoid an error from the lidar that make front value blinking and cause the robot to go backward
        # because it think there is an obstacle in front of him
        array_lidar = [0]*360
        for scan in self.lidar.iter_scans(scan_type='express') :
            # array_lidar = [0]*360

            # Print the number of points in the scan for debug
            self.get_logger().debug(f"nb pts : {len(scan)}")
            # Store the scan data in the array
            for i in range(len(scan)) :
                angle = min(359,max(0,359-int(scan[i][1]))) #scan[i][1]:angle
                array_lidar[angle]=scan[i][2] / 1000 #scan[i][2]:distance in mm -> m

            # Lock the varaible to avoid publishing with wrong time stamp
            with self.data_lock:
                # Add time stamp and data to the LidarScan object
                self.lidar_data.header.stamp = self.get_clock().now().to_msg()
                self.lidar_data.ranges = array_lidar

            # Check the stop signal after each scan
            if self.stop_thread.is_set():
                return
      
    def start_publish(self):
        rate = self.create_rate(self.publish_rate)  # Define the rate object with your desired rate in Hz

        try:
            rate = self.create_rate(self.publish_rate)
            while rclpy.ok():
                # Publish data in lidar_data
                self.pub.publish(self.lidar_data)
                rate.sleep()

        except (RPLidarException, ValueError, ExternalShutdownException) as e: #normallement ROSInterruptException mais doit trouver équivalence
            self.get_logger().info(f"an error occured: {e}")
        finally:
            # If an exception occurs (mainly because of ROSInterruptException)
            # Signal the thread to stop
            self.stop_thread.set()
            # Wait for the thread to finish
            self.collect_thread.join()

            # shut down the lidar
            self.get_logger().info("[INFO] -- shutting down lidar...")
            self.lidar.stop_motor()
            self.lidar.stop()
            time.sleep(1)
            self.lidar.disconnect()
            self.get_logger().info("[INFO] -- lidar shutted down")
            exit(0)

def main(args=None):
    rclpy.init(args=args)

    try:
        # Create a LidarPublisher and start it
        lidarcontrol = LidarPublisher()
    except ExternalShutdownException: # normalement ROSInterruptExcption mais je dois trouver équivalence
        # If a ROSInterruptException occurs, exit the program
        exit(0)
