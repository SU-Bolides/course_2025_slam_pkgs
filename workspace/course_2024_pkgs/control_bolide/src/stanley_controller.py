#!/usr/bin/env python3

__author__ = "Nicolas HAMMJE"
__status__ = "In Developpement"

#%% IMPORTS
import rospy
import math
import copy
from std_msgs.msg import Float32MultiArray
from control_bolide.msg import SpeedDirection
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from perception_bolide.msg import MultipleRange
from sensor_msgs.msg import LaserScan, Imu
from visualization_msgs.msg import Marker, MarkerArray

from time import perf_counter_ns

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
# from scipy import signal

from inputimeout import inputimeout, TimeoutOccurred





#%% CLASS
class StanleyController:
    '''
    This class implements a stanley controller and (soon) an obstacle avoidance system. 
    '''
    def __init__(self):
        rospy.init_node('stanley_controller_node')
        
        self.WAYPOINTS_PATH = str(rospy.get_param("~waypoints_path", "~/bolide_ws/course_2024_pkgs/control_bolide/racelines/esclangon_couloir_reverse.csv"))
        # self.scan_topic = str(rospy.get_param("~scan_topic", "lidar_data"))
        self.ODOM_TOPIC = str(rospy.get_param("~odom_topic", "/pf/pos/odom"))
        self.CMD_TOPIC = str(rospy.get_param("~cmd_topic", "cmd_vel"))
        
        self.turn_around = False
        
        self.started = False
        
        self.K_E = float(rospy.get_param("~K_E", "2.0"))
        self.K_H = float(rospy.get_param("~K_H", "1.5"))
        self.K_V = float(rospy.get_param("~K_V", "0.5"))
        self.K_p_obstacle = float(rospy.get_param("~K_p_obstacle", "0.5"))
        self.K_dh = float(rospy.get_param("~K_dh", "0.01"))
        self.K_ds = float(rospy.get_param("~K_ds", "0.0"))

        rospy.loginfo(self.K_E)

        self.VELOCITY_PERCENTAGE = float(rospy.get_param("~velocity_percentage", "0.3"))
        self.STEERING_LIMIT_DEG = float(rospy.get_param("~steering_limit_deg", "15.2"))
        self.BRAKE_THRESHOLD_MS = float(rospy.get_param("~brake_threshold_ms", "0.5"))


        # init publisher
        self.drive_pub = rospy.Publisher(self.CMD_TOPIC, SpeedDirection, queue_size=10)
        self.current_waypoint_pub = rospy.Publisher("current_waypoint", Marker, queue_size=10)
        self.waypoint_pub = rospy.Publisher("next_waypoint", Marker, queue_size=10)
        self.diag_pub = rospy.Publisher("/stanley_diagnostics", Float32MultiArray, queue_size=1)

        self.stanley_avoidance_path_pub = rospy.Publisher("stanley_avoidance", Marker,queue_size=10)
        self.stanley_avoidance_path_array_pub = rospy.Publisher("stanley_avoidance_path",MarkerArray, queue_size=10)

        self.t1_start = perf_counter_ns()

        self.laser_scan = None 
        self.car_width = 0.4

        self.commanded_velocity = 0
        self.speed_ackermann = 0
        self.counter = 0 

        self.old_steering_angle = 0 
        self.curr_steering_angle = 0


        self.car_state_sub = rospy.Subscriber("car_state", SpeedDirection, self.carstateCB, queue_size=1)
        # self.timer = rospy.Timer(rospy.Duration(0.4), self.timer_callback)

        self.starting_check_timer = rospy.Timer(rospy.Duration(0.5), self.starting_check_timer)
        self.stuck_timer = rospy.Timer(rospy.Duration(0.10), self.check_stuck)

        self.waypoint_utils = WaypointUtils(
            node=self,
            interpolation_distance=0.1,
            filepath=self.WAYPOINTS_PATH
        )

        self.utils = Utils()

        self.old_path_heading = [None, None]

        self.current_rear_distance = 0.3


        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.target_velocity = 0.0
        self.old_target_vel = 0.0
        self.old_target_heading = 0.0

        self.going_backwards = 0

        self.gp_none = 0

        self.rear_index = 0
        self.front_index = 1

        self.obstacle_detected = False

        self.old_error = None
        self.old_crosstrack_error = None

        self.reversing = False

        self.started = False #START CAR NAV

        self.current_pose = None
        self.current_pose_wheelbase_front = None
        self.goal_pos = None
        self.closest_wheelbase_rear_point = None

        self.cells_per_meter = 25

        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odomCB, queue_size=10)
        self.acker_sub = rospy.Subscriber("ackermann_odom", Odometry, self.ackerCB, queue_size=10)
        # self.imu_sub = rospy.Subscriber("raw_imu_data", Imu, self.imuCB, queue_size=10)
        self.laser_scan_sub = rospy.Subscriber("lidar_data", LaserScan, self.laser_scan_cb, queue_size=10)
        self.infrared_sub = rospy.Subscriber("raw_rear_range_data", MultipleRange, self.rear_range_cb, queue_size=10)


    def carstateCB(self, msg):
        self.old_steering_angle = self.curr_steering_angle
        self.curr_steering_angle = msg.direction


    def destroy_node(self):
        kill = SpeedDirection()
        kill.speed = 0
        kill.direction = 0
        self.drive_pub.publish(kill)

    def rear_range_cb(self, msg):
        self.current_rear_distance = np.min([msg.IR_rear_left.range, msg.IR_rear_right.range])
        pass

    def laser_scan_cb(self, msg):
        #Laser scan callback.
        # print("LASER CB")
        if self.laser_scan is None:
            print("Got first lidar scan")
            self.lidar_angle_min = msg.angle_min
            self.lidar_angle_max = msg.angle_max
            self.lidar_angle_incr = msg.angle_increment
            self.angles = np.arange(self.lidar_angle_min, self.lidar_angle_max+1, self.lidar_angle_incr)

        self.laser_scan = np.array(msg.ranges)
        self.laser_scan[self.laser_scan == np.inf] = 16.

    def check_stuck(self, _):
        # Checks if commanded speed is pos and .  
        # print("started?")  
        if self.started:                
            if abs(self.speed_ackermann) < 0.1 and not self.reversing:
                self.counter += 1 
                # print("not moving.....???", self.counter)
                if self.counter >= 10: 

                    front =  self.laser_scan[int(np.shape(self.laser_scan)[0] * 0.5)]%16.0
                    right = self.laser_scan[0]%16.0
                    left = self.laser_scan[int(np.shape(self.laser_scan)[0]) - 1]%16.0



                    if front < 0.3 or right < 0.3 or left < 0.3:
                        # print("REVERSING, THIS SHIT AINT NORMAL MY G")
                        self.reversing = True
                        self.reverse()
                    else:
                        if self.counter >= 12:
                            # print("I'm reversing, something ain't right in this btch")
                            self.reversing = True
                            self.reverse()
            else:
                # print("not stuck", self.speed_ackermann, self.reversing)
                if self.reversing:
                    self.counter = max(self.counter-1.0, 0)
                    if not self.counter:
                        self.reversing = False
                    self.reverse()
        else:
            pass
            # print("stuck not started")
            # print(self.started)


            
    

    def reverse(self):
        try:
            angle_to_waypoint = math.atan2(self.goal_pos[1], self.goal_pos[0])
            direction = -1*(angle_to_waypoint / abs(angle_to_waypoint)) 
        except:
            direction = - (self.current_direction / abs(self.current_direction + 1e-6)) * self.STEERING_LIMIT_DEG

        if self.turn_around:
            direction *= -1

        speed = -10 * ((self.current_rear_distance+0.1)/ 0.3) - 0.33  
        self.publish_cmd_vel(speed, direction)

    def ackerCB(self, msg):
        self.speed_ackermann = msg.twist.twist.linear.x
        self.curr_ang_vel = msg.twist.twist.angular.z

    def starting_check_timer(self, _):
        #Check to start:
        try:
            print("Car is ready to go. Please ensure all checks are completed.")
            c = inputimeout(prompt='Have you checked ESC is on, init pose is set in RViz, correct map is loaded, launched rosbag record? y/n\n', timeout=15)
        except TimeoutOccurred:
            c = 'n'
        if c == 'y':
            print("GODSPEED <3")
            print("Starting car")
            self.started = True
            self.starting_check_timer.shutdown()




    ###TAKEN FROM STEVEN GONG

    def emergency_brake(self):
        print("EMERGENCY BRAKE")
        angle = 0
        velocity = 0.0
        self.publish_cmd_vel(velocity, angle)


    def check_collision(self, goal_pose):

        t1_start = perf_counter_ns()

        if goal_pose is None:
            rospy.logwarn("No more checkpoints")
            self.gp_none += 1
            self.drive_to_target_stanley()
            return False, 0
        else:
            self.gp_none -= 1
        
        if self.gp_none>5:
            self.emergency_brake()

        angle_to_waypoint = math.atan2(goal_pose[1], goal_pose[0])
        angle_to_waypoint_ranges = angle_to_waypoint / self.lidar_angle_incr

        dist = math.sqrt(goal_pose[0]**2 + goal_pose[1]**2)
        # print("Dist", dist)
        angle = math.atan2(self.car_width*0.5, dist) #We want to check the distance on this twice

        center_pos = (np.shape(self.laser_scan)[0] * 0.5) + angle_to_waypoint_ranges
        mean_front = np.mean(self.laser_scan[int(center_pos - angle/self.lidar_angle_incr) : int(center_pos+angle/self.lidar_angle_incr)])

        # print(angle_to_waypoint)
        # print("mean_front", mean_front)

        # if (np.sum(self.laser_scan[int(center_pos - angle/self.lidar_angle_incr) : int(center_pos+angle/self.lidar_angle_incr)] < dist)) > 5:
            
        #     print("dist", dist)
        #     print("Obstacle front!", mean_front)

        #     left_angle = min(max(- 0.1 - angle_to_waypoint, -0.27), 0.27)
        #     left_pos = center_pos + left_angle/self.lidar_angle_incr
        #     left_dist = dist / math.cos(left_angle)

        #     obs_left = False
        #     mean_left = np.mean(self.laser_scan[int(left_pos - angle/self.lidar_angle_incr) : int(left_pos+angle/self.lidar_angle_incr)])
        #     if (np.sum(self.laser_scan[int(left_pos - angle/self.lidar_angle_incr) : int(left_pos+angle/self.lidar_angle_incr)] < left_dist)) > 8:
        #         print("OBSTACLE LEFT", mean_left)
        #         obs_left = True

        #     right_angle = min(max(0.1 + angle_to_waypoint, -0.27), 0.27)
        #     right_pos = center_pos + right_angle/self.lidar_angle_incr
        #     right_dist = dist / math.cos(right_angle)

        #     obs_right = False
        #     mean_right = np.mean(self.laser_scan[int(right_pos - angle/self.lidar_angle_incr) : int(right_pos+angle/self.lidar_angle_incr)])
        #     if (np.sum(self.laser_scan[int(right_pos - angle/self.lidar_angle_incr) : int(right_pos+angle/self.lidar_angle_incr)] < right_dist)) > 8:
        #         print("OBSTACLE RIGHT", mean_right)
        #         obs_right = True



        #     steering_angle = ((self.K_p_obstacle * (mean_left - mean_right)) / mean_front) * self.STEERING_LIMIT_DEG
        #     # print("1", steering_angle)
        #     steering_angle += math.degrees(obs_right * 0.10 - obs_left * 0.10)
        #     # print("2", steering_angle)
        #     steering_angle = min(self.STEERING_LIMIT_DEG, max(steering_angle, -self.STEERING_LIMIT_DEG))
        #     # print("3", steering_angle)
            
        #     # print("Elapsed time collision check:",(perf_counter_ns()-t1_start) * 1e-6, 'ms')

        #     return True, steering_angle
            

            
        
        # print("Elapsed time collision check:",(perf_counter_ns()-t1_start) * 1e-6, 'ms')

        return False, 0
        


    def check_for_obstacles(self):
        t1_start = perf_counter_ns()

        if self.laser_scan is None:
            print("No laser scan yet")
            self.drive_to_target_stanley()
            return

        obs, angle = self.check_collision(self.goal_pos)

        # print("Elapsed time obstacle check:",(perf_counter_ns()-t1_start) * 1e-6, 'ms')

        if obs:
            if abs(angle) < 3:
                #Why even bother with the drive around lmao just follow the waypoints and hope for the best my g
                self.drive_to_target_stanley()
            else:
                self.drive_around(angle)
        else:
            self.drive_to_target_stanley()

        
        


    def drive_around(self, angle):
        print("DRIVE AROUND")
        #The input is a point in the car's frame of reference. ie if the point is 10,10, it will be 10 m ahead and 10 m to the right of the car. 
        #This makes computing distances and angles trivial

        # determine velocity
        if angle < 5:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.7
        elif angle < 10.0:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.2
        else:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.0
        print(angle)
        self.publish_cmd_vel(velocity, -angle)


    def drive_to_target_stanley(self):

        closest_wheelbase_front_point_car, closest_wheelbase_front_point_world, self.front_index = self.waypoint_utils.get_waypoint_stanley(
            self.current_pose_wheelbase_front
        )  


        path_heading = math.atan2(
            closest_wheelbase_front_point_world[1] - self.closest_wheelbase_rear_point[1],
            closest_wheelbase_front_point_world[0] - self.closest_wheelbase_rear_point[0],
        )

        current_heading = math.atan2(
            self.current_pose_wheelbase_front.position.y - self.current_pose.position.y,
            self.current_pose_wheelbase_front.position.x - self.current_pose.position.x,
        )     

        # print("FRONT, REAR ", self.front_index, self.rear_index)
        if (self.front_index+2 < self.rear_index) and not ((self.rear_index - self.front_index) > (self.waypoint_utils.nb_points - 10)):
            self.going_backwards +=1
            if self.going_backwards > 3:
                rospy.logwarn("GOING BACKWARDS!!")
                print(self.front_index, self.rear_index)
                self.target_velocity *= 0.3
                current_heading += math.pi
                self.turn_around = True
        else: 
            self.going_backwards -= 1
            self.turn_around = False



        if path_heading < 0:
            path_heading += 2 * math.pi
        if current_heading < 0:
            current_heading += 2 * math.pi


        steer_damper = self.old_steering_angle - self.curr_steering_angle
        steer_damper *= self.K_ds

        rate_err = (self.target_curvature * self.speed_ackermann) - self.curr_ang_vel
        rate_err *= self.K_dh


        # print("Target, curr ang vel", (self.target_curvature * self.speed_ackermann), self.curr_ang_vel)

        # rospy.loginfo(f"rate error: {rate_err:.2f}")
        
        self.old_path_heading = [path_heading, rospy.Time.now()]

        # calculate the errors
        crosstrack_error = math.atan2(
            self.K_E * closest_wheelbase_front_point_car[1], self.K_V + self.target_velocity
        )  # y value in car frame
        heading_error = path_heading - current_heading
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi
 
        heading_error *= self.K_H

        # Calculate the steering angle using the Stanley controller formula
        angle = heading_error + crosstrack_error + rate_err + steer_damper


        # print("Heading, crosstrack, rate, damper", heading_error, crosstrack_error, rate_err, steer_damper)

        debug = Float32MultiArray()
        debug.data = [heading_error, crosstrack_error, rate_err]
        self.diag_pub.publish(debug)

        rospy.loginfo(f"heading_error: {heading_error:.2f} ... crosstrack_error: {crosstrack_error:.2f} angle: {np.degrees(angle):.2f}")
        rospy.loginfo(f"current_heading: {current_heading:.2f} ... path_heading: {path_heading:.2f}")

        angle = np.clip(angle, -math.radians(self.STEERING_LIMIT_DEG), math.radians(self.STEERING_LIMIT_DEG))

            

        velocity = self.target_velocity * self.VELOCITY_PERCENTAGE

        # if abs(angle) > 0.24:
        #     velocity *= 0.7


        # print("angle", angle)

        self.publish_cmd_vel(velocity, angle)


    def publish_cmd_vel(self, velocity, angle):
        if not rospy.is_shutdown():
            drive_msg = SpeedDirection()


            ## If the speed difference is bigger than the brake_threshold, we want to apply the brakes
            # rospy.logwarn((self.target_velocity - self.old_target_vel))
            if ((self.target_velocity - self.old_target_vel)) < -self.BRAKE_THRESHOLD_MS:
                rospy.logwarn("BRAKING BRAKING BRAKING BRAKING BRAKING")
                velocity = 2.0
                

            #Max Speed is 7 m/s
            if not velocity == 2.0:
                velocity_normalized = velocity / 10.0
                velocity_normalized = min(max(velocity_normalized, -1), 1.0)
            else:
                velocity_normalized = 2.0

            drive_msg.speed = velocity_normalized

            self.commanded_velocity = velocity
        
            angle_normalized = angle / np.radians(self.STEERING_LIMIT_DEG)

            drive_msg.direction = -angle_normalized
            self.drive_pub.publish(drive_msg)


    def imuCB(self, imudata: Imu):
        self.curr_ang_vel = imudata.angular_velocity.z


    
    def odomCB(self, pose_msg: Odometry):
        # print("Elapsed time ODOM:",(perf_counter_ns()-self.t1_start) * 1e-6, 'ms') 
        self.t1_start = perf_counter_ns()


        self.current_pose = pose_msg.pose.pose
        
        current_pose_quaternion = np.array(
            [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w,
            ]
        )

        self.current_pose_wheelbase_front = Pose()
        current_pose_xyz = R.from_quat(current_pose_quaternion).apply((0.195,0,0)) + (
            self.current_pose.position.x,
            self.current_pose.position.y,
            0,
        )

        self.current_pose_wheelbase_front.position.x = current_pose_xyz[0]
        self.current_pose_wheelbase_front.position.y = current_pose_xyz[1]
        self.current_pose_wheelbase_front.position.z = current_pose_xyz[2]
        self.current_pose_wheelbase_front.orientation = self.current_pose.orientation

        self.old_target_vel = self.target_velocity

        self.closest_wheelbase_rear_point, self.target_velocity, self.rear_index, self.target_curvature = self.waypoint_utils.get_closest_waypoint_with_velocity(
            self.current_pose
        )

        self.utils.draw_marker(
            pose_msg.header.frame_id,
            pose_msg.header.stamp,
            self.closest_wheelbase_rear_point,
            self.current_waypoint_pub,
            color="blue",
        )

        # print("front", self.front_index)
        if self.turn_around:
            self.goal_pos, goal_pos_world = self.waypoint_utils.get_waypoint(self.current_pose, self.target_velocity*self.VELOCITY_PERCENTAGE, self.front_index, bw=True)
        else:
            self.goal_pos, goal_pos_world = self.waypoint_utils.get_waypoint(self.current_pose, self.target_velocity*self.VELOCITY_PERCENTAGE, self.front_index)
        # self.goal_pos = [2.1, 0.2]

        self.utils.draw_marker(pose_msg.header.frame_id, pose_msg.header.stamp, goal_pos_world, self.waypoint_pub, color="red")

            
        

        if self.started:
            if self.reversing:
                self.reverse()
            else:
                self.check_for_obstacles()
                #self.drive_to_target_stanley()
        else:
            pass
            # print("not started")


class Utils:
    def __init__(self):
        pass

    def draw_marker(self, frame_id, stamp, position, publisher, color="red", id=0):
        if position is None:
            return
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.id = id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if color == "red":
            marker.color.r = 1.0
        elif color == "green":
            marker.color.g = 1.0
        elif color == "blue":
            marker.color.b = 1.0
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 0.0
        publisher.publish(marker)



    def draw_marker_array(self, frame_id, stamp, positions, publisher):
        marker_array = MarkerArray()
        for i, position in enumerate(positions):
            if position is None:
                continue
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = 0.0
            marker_array.markers.append(marker)
        publisher.publish(marker_array)



    def draw_lines(self, frame_id, stamp, path, publisher):
        points = []
        for i in range(len(path) - 1):
            a = path[i]
            b = path[i + 1]
            point = Point()
            point.x = a[0]
            point.y = a[1]
            points.append(copy.deepcopy(point))
            point.x = b[0]
            point.y = b[1]
            points.append(copy.deepcopy(point))

        line_list = Marker()
        line_list.header.frame_id = frame_id
        line_list.header.stamp = stamp
        line_list.id = 0
        line_list.type = line_list.LINE_LIST
        line_list.action = line_list.ADD
        line_list.scale.x = 0.1
        line_list.color.a = 1.0
        line_list.color.r = 0.0
        line_list.color.g = 1.0
        line_list.color.b = 0.0
        line_list.points = points
        publisher.publish(line_list)


    


    def traverse_grid(self, start, end):
        """
        Bresenham's line algorithm for fast voxel traversal

        CREDIT TO: Rogue Basin
        CODE TAKEN FROM: http://www.roguebasin.com/index.php/Bresenham%27s_Line_Algorithm
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1

        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)

        # Rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2

        # Swap start and end points if necessary and store swap state
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1

        # Calculate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1

        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        return points

    
class WaypointUtils:
    def __init__(self, node, filepath, interpolation_distance):
        self.interp_distance = interpolation_distance
        self.node = node

        self.waypoints_world, self.velocities, self.curvatures = self.load_and_interpolate_waypoints(
            file_path=filepath, interpolation_distance=interpolation_distance, reverse=False
        )

        self.nb_points = len(self.waypoints_world)

        self.index = 0 
        self.velocity_index = 0
        self.heading_index = 0

        self.curr_index = None
        

        self.min_lookahead = 0.25
        self.max_lookahead = 3.0

        self.min_lookahead_speed = 2.0
        self.max_lookahead_speed = 6.0


        print(f"Loaded {len(self.waypoints_world)} waypoints")


    def transform_waypoints(self, waypoints, car_position, pose):
        # translation
        waypoints = waypoints - car_position

        # rotation
        quaternion = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        waypoints = R.inv(R.from_quat(quaternion)).apply(waypoints)

        return waypoints

    def get_closest_waypoint_with_velocity(self, pose):
        # get current position of car
        if pose is None:
            return

        position = (pose.position.x, pose.position.y, 0)

        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
    
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        self.velocity_index = np.argmin(distances)

        self.curr_index = self.velocity_index

        return self.waypoints_world[self.velocity_index], self.velocities[self.velocity_index], self.velocity_index, self.curvatures[self.velocity_index]

    def get_waypoint(self, pose, target_velocity, curr_index, fixed_lookahead=None, bw=False):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        
        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
       
        # get distance from car to all waypoints
        if fixed_lookahead:
            self.L = fixed_lookahead
        else:
            # Lookahead is proportional to velocity
            self.L = min(
                max(
                    self.min_lookahead,
                    self.min_lookahead
                    + ((self.max_lookahead - self.min_lookahead) * ((target_velocity) - self.min_lookahead_speed) )/ (self.max_lookahead_speed - self.min_lookahead_speed),
                ),
                self.max_lookahead,
            )
        
        if self.curr_index is None:
            #if we don't have a current idea of the index, just get the next one like we always did.
            distances = np.linalg.norm(waypoints_car, axis=1)

            if bw:
                indices_L = np.argsort(np.where(distances < self.L, distances, -1))

                # print(self.L)

                # set goal point to be the farthest valid waypoint within distance L
                for i in indices_L:
                    # check waypoint is behind the car
                    x = waypoints_car[i][0]
                    if x < 0:
                        self.index = i
                        if self.index > 124:
                            self.index -= 124
                        return waypoints_car[self.index+5], self.waypoints_world[self.index+5]
            else:   
                indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]
                # print(self.L)
                # set goal point to be the farthest valid waypoint within distance L
                for i in indices_L:
                    # check waypoint is in front of car
                    x = waypoints_car[i][0]
                    if x > 0 and i < curr_index + 20:
                        self.index = i
                        if self.index > 124:
                            self.index -= 124
                        return waypoints_car[self.index+5], self.waypoints_world[self.index+5]
        else:

            index_step = int(self.L / self.interp_distance)
            if bw:
                next_waypoint = (self.curr_index - index_step) % len(self.waypoints_world)
            else:
                next_waypoint = (self.curr_index + index_step) % len(self.waypoints_world)

            self.index = next_waypoint
            if self.index > 124:
                self.index -= 124
            return waypoints_car[self.index+5], self.waypoints_world[self.index+5]




        return None, None


    def get_waypoint_stanley(self, pose):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        
        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
        
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints sorted by ascending distance
        index = np.argmin(distances)

        if index > 124:
            index = index - 124

        return waypoints_car[index+5], self.waypoints_world[index+5], index+5


    def load_and_interpolate_waypoints(self, file_path, interpolation_distance=0.2, reverse=False):
        # Read waypoints from csv, first two columns are x and y, third column is velocity
        # Exclude last row, because that closes the loop
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]
        velocities = np.genfromtxt(file_path, delimiter=",")[:, 2]
        curvature = np.genfromtxt(file_path, delimiter=",")[:, 3]

        if reverse:
            points = points[::-1]
            velocities = velocities[::-1]
            curvature = curvature[::-1]


        # yaw_rate = velocities * curvature

        # Add first point as last point to complete loop
        rospy.loginfo("Velocities: " + str(velocities))
        rospy.loginfo("Curvature: " + str(curvature))

        # interpolate, not generally needed because interpolation can be done with the solver, where you feed in target distance between points
        if interpolation_distance != 0 and interpolation_distance is not None:
            # Calculate the cumulative distances between points
            print("INTERPOLATING")
            distances = np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1))
            cumulative_distances = np.insert(np.cumsum(distances), 0, 0)

            # Calculate the number of segments based on the desired distance threshold
            total_distance = cumulative_distances[-1]
            segments = int(total_distance / interpolation_distance)

            # Linear length along the line
            distance = np.cumsum(np.sqrt(np.sum(np.diff(points, axis=0) ** 2, axis=1)))
            # Normalize distance between 0 and 1
            distance = np.insert(distance, 0, 0) / distance[-1]

            # Interpolate
            alpha = np.linspace(0, 1, segments)
            interpolator = interp1d(distance, points, kind="slinear", axis=0)
            interpolated_points = interpolator(alpha)

            # Interpolate velocities
            velocity_interpolator = interp1d(distance, velocities, kind="slinear")
            interpolated_velocities = velocity_interpolator(alpha)

            curvature_interpolator = interp1d(distance, curvature, kind="slinear")
            interpolated_curvature = curvature_interpolator(alpha)

            # Add z-coordinate to be 0
            interpolated_points = np.hstack((interpolated_points, np.zeros((interpolated_points.shape[0], 1))))
            assert len(interpolated_points) == len(interpolated_velocities)
            assert len(interpolated_curvature) == len(interpolated_velocities)
            return interpolated_points, interpolated_velocities, interpolated_curvature

        else:
            # Add z-coordinate to be 0
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
            return points, velocities, curvature


#%% MAIN
if __name__ == '__main__':
    print("Stanley Avoidance Initialized")
    controller = StanleyController()
    
    try: 
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt received. Shutting down...")
    finally:
        # Clean up and unregister keyboard events
        controller.destroy_node()