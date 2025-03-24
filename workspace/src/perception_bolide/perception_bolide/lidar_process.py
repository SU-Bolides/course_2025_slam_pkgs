import numpy as np
from scipy import signal

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

class LidarProcess(Node):
    def __init__(self):
        super().__init__('lidar_process')

        self.get_logger().info("[INFO] -- Initializing the lidar process data node")

        # Store last arrays containing lidar data
        self.last_values = []

        # Set the parameters :
        self.declare_parameter('temporal_filter_bool', False)
        self.declare_parameter('spatial_filter_bool', False)
        self.delcare_parameter('anti_jumping_filter_bool', False)
        self.declare_parameter('spatial_filter_range', 1)
        self.declare_parameter('temporal_filter_range', 5)
        self.declare_parameter('anti_jumping_filter_range', 5)

        # all the following parameters could be set in the launch file
        self.declare_parameter('lidar_min_angle_deg', -90) # in degrees
        self.declare_parameter('lidar_max_angle_deg', 90)
        self.old_min = self.get_parameter('lidar_min_angle_deg').get_parameter_value().int_value
        self.old_max = self.get_parameter('lidar_max_angle_deg').get_parameter_value().int_value # we store this 2 variables for futur utilities in callback function
        self.declare_parameter('lidar_min_angle_rad', self.old_min*np.pi/180)
        self.declare_parameter('lidar_max_angle_rad', self.old_max*np.pi/180)

        #occupancy grid:
        self.LOOKAHEAD = 4 #meters
        self.CELLS_PER_METER = 25
        self.IS_FREE = 0
        self.IS_OCCUPIED = 100
        self.grid_height = self.LOOKAHEAD * self.CELLS_PER_METER
        self.grid_width = 2 * self.CELLS_PER_METER

        # SUBSCRIBER
        self.sub_raw_lidar_data = self.create_subscription(LaserScan, '/raw_lidar_data', self.callback, 10)
        self.sub_param_change = self.create_subscription(Bool, '/param_change_alert', self.callback_parameters, 10)
        # PUBLISHER
        self.pub = self.create_publisher(LaserScan, '/lidar_data', 10)
        self.occupancy_grid_pub = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

    def populate_occupancy_grid(self, ranges, angle_increment):
        # reset empty occupacny grid (-1 = unknown)

        self.occupancy_grid = np.full(shape=(self.grid_height, self.grid_width), fill_value=self.IS_FREE, dtype=int)

        ranges = np.array(ranges)
        indices = np.arange(len(ranges))
        thetas = (indices * angle_increment) + self.min_angle_rad
        xs = ranges * np.cos(thetas)
        ys = ranges * np.sin(thetas)

        i = np.round(xs * -self.CELLS_PER_METER + (self.grid_height - 1)).astype(int)
        j = np.round(ys * -self.CELLS_PER_METER + ((self.grid_width // 2) - 1)).astype(int)

        occupied_indices = np.where((i >= 0) & (i < self.grid_height) & (j >= 0) & (j < self.grid_width))
        self.occupancy_grid[i[occupied_indices], j[occupied_indices]] = self.IS_OCCUPIED


        kernel = np.ones(shape=[2, 2])
        self.occupancy_grid = signal.convolve2d(
            self.occupancy_grid.astype("int"), kernel.astype("int"), boundary="symm", mode="same"
        )
        self.occupancy_grid = np.clip(self.occupancy_grid, -1, 100)

    def publish_occupancy_grid(self):
        """
        Publish populated occupancy grid to ros2 topic
        Args:
            scan_msg (LaserScan): message from lidar scan topic
        """
        oc = OccupancyGrid()
        oc.header.frame_id = self.frame_id
        oc.header.stamp = self.stamp
        oc.info.origin.position.y -= ((self.grid_width / 2) + 1) / self.CELLS_PER_METER
        oc.info.width = self.grid_height
        oc.info.height = self.grid_width
        oc.info.resolution = 1 / self.CELLS_PER_METER
        oc.data = np.fliplr(np.rot90(self.occupancy_grid, k=1)).flatten().tolist()
        self.occupancy_grid_pub.publish(oc)

    def callback_parameters(self, data:Bool) :

        # When we change the range of angles, it can cause problem for temporal median filtering
        # We must then reinitialize the last values stored
        min_angle_deg = self.get_parameter('min_angle_deg').get_parameter_value().int_value
        max_angle_deg = self.get_parameter('max_angle_deg').get_parameter_value().int_value

        if self.old_min != min_angle_deg or self.old_max != max_angle_deg :
            self.last_values = []

        try : self.iii += 1
        except : self.iii = 1
        self.get_logger().debug(f"[DEBUG] -- Retrieved parameters for the {self.iii}th time")

        self.old_min = min_angle_deg
        self.old_max = max_angle_deg

    def callback(self, data:LaserScan) :
        """ Callback function called when a message is received on the subscribed topic"""
        print(len(data.ranges))
        # Check that the data array has a length of 360
        if not (len(data.ranges) == 1153):
            self.get_logger().debug("the lidar array is not composed of 1153 values")
            self.get_logger().debug(len(data.ranges))
            return

        min_angle_deg = self.get_parameter('min_angle_deg').get_parameter_value().int_value
        max_angle_deg = self.get_parameter('max_angle_deg').get_parameter_value().int_value
        min_angle_rad = self.get_parameter('min_angle_rad').get_parameter_value().int_value
        max_angle_rad = self.get_parameter('max_angle_rad').get_parameter_value().int_value
        # Check that the min angle is less than the max angle
        if not min_angle_deg < max_angle_deg:
            self.get_logger().warn("The min angle must be inferior to the max angle")
            return

        # Check that the min and max angles are integers
        if not (isinstance(min_angle_deg, int) and isinstance(max_angle_deg, int)):
            self.get_logger().error("The min and max angles must be integers")
            return

        # crop the data
        cropped_data = self.crop_data(data.ranges, data.angle_increment)

        # apply filters to data
        data_filtered = self.filter_lidar(cropped_data)

        # retrieving data from the LidarSensor msg

        self.stamp = data.header.stamp
        self.frame_id = "lidar_frame"

        lidar_data = LaserScan()
        lidar_data.header.stamp = self.stamp
        lidar_data.header.frame_id = self.frame_id
        lidar_data.angle_min = min_angle_rad # in radians
        lidar_data.angle_max = max_angle_rad # in radians
        lidar_data.angle_increment = 0.00545 # in radians (should be 1 degree)
        lidar_data.time_increment = 0.0002 #data.time_increment # in seconds
        lidar_data.scan_time = 0.072 # in seconds
        lidar_data.range_min = data.range_min # in meters
        lidar_data.range_max = data.range_max # in meters

        lidar_data.ranges = data_filtered

        self.populate_occupancy_grid(data_filtered, data.angle_increment)
        self.publish_occupancy_grid()

        self.pub.publish(lidar_data)

    def crop_data(self, data:list, angle_increment) :

        min_angle_rad = self.get_parameter('min_angle_rad').get_parameter_value().int_value
        max_angle_rad = self.get_parameter('max_angle_rad').get_parameter_value().int_value

        angle_min_crop = min_angle_rad
        angle_max_crop = max_angle_rad


        angle_min = -3.1415926535
        angle_max = 3.1415926535
        ranges = np.roll(np.array(data), int(len(data)/2)) #-pi/2 is first

        # end_index = int(round((angle_max_crop*2) / angle_increment))

        # cropped_ranges = ranges[:end_index]

        # Calculate start and end indices for cropping
        start_index = int((angle_min_crop - angle_min) / angle_increment)
        end_index = int((angle_max_crop - angle_min) / angle_increment)

        # Ensure indices are within the range of available data
        start_index = max(0, min(start_index, len(ranges)))
        end_index = max(0, min(end_index, len(ranges)))

        # Crop the range data
        cropped_ranges = ranges[start_index:end_index+1]

        return list(cropped_ranges)

    def filter_lidar(self, data:list):
        """Filter the lidar data using a median filter"""

        # Convert the data list to a numpy array
        data_array = np.array(data)

        spatial_filter = self.get_parameter('spatial_filter_bool').get_parameter_value().bool_value
        temporal_filter = self.get_parameter('temporal_filter_bool').get_parameter_value().bool_value
        anti_jumping_filter = self.get_parameter('anti_jumping_filter_bool').get_parameter_value().bool_value
        spatial_filter_range = self.get_parameter('spatial_filter_range').get_parameter_value().int_value
        temporal_filter_range = self.get_parameter('temporal_spatial_range').get_parameter_value().int_value
        anti_jumping_filter_range = self.get_parameter('anti_jumping_filter_range').get_parameter_value().int_value

        if spatial_filter :
            # Spatial median filter:
            # Copy the original data array
            final_array = np.copy(data_array)

            # For each value in the spatial filter range, shift the data array by that amount in both directions,
            # stack the shifted arrays on top of the final array
            for i in range(1, spatial_filter_range + 1) :
                final_array = np.vstack([final_array, np.roll(data_array, i)])
                final_array = np.vstack([final_array, np.roll(data_array, -i)])

            # Replace the original data array with the median of the final array
            data_array = np.median(final_array, axis = 0)

        if temporal_filter :
            # Temporal median filter:
            # Add the current data array to the list of last values
            self.last_values += [data_array]

            # If the list of last values is longer than the temporal filter range, remove the oldest values
            while len(self.last_values) > temporal_filter_range : self.last_values.pop(0)

            # Convert the list of last values to a numpy array
            data_array = np.array(self.last_values)

            # Replace the original data array with the median of the last values
            data_array = np.median(data_array, axis = 0)

        if anti_jumping_filter :
            # Anti-jumping filter to prevent unexpected jumps in the lidar data:
            # For each null value in the data array, replace it with the median of the values around it (non-null values) (around in a temporal sense)

            # Add the current data array to the list of last values
            self.last_values += [data_array]

            # If the list of last values is longer than the temporal filter range, remove the oldest values
            while len(self.last_values) > temporal_filter_range : self.last_values.pop(0)

            # where the data array is null, replace it with the previous non-null value (t-1)
            if len(self.last_values) > 1 :
                data_array[data_array == 0] = self.last_values[-2][data_array == 0]

        # Return the filtered data array as a list
        return list(data_array)

def main(args=None):
    rclpy.init(args=args)

    lidar_process = LidarProcess()

    try:
        rclpy.spin(lidar_process)
    except ExternalShutdownException:
        exit(0)
    