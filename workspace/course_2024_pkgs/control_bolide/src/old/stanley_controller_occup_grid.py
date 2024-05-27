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
from nav_msgs.msg import Odometry, OccupancyGrid
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
        
        
        
        self.K_E = float(rospy.get_param("~K_E", "2.0"))
        self.K_H = float(rospy.get_param("~K_H", "1.5"))
        self.K_V = float(rospy.get_param("~K_V", "0.5"))
        self.K_p_obstacle = float(rospy.get_param("~K_p_obstacle", "0.5"))
        self.K_dh = float(rospy.get_param("~K_dh", "0.01"))
        self.K_ds = float(rospy.get_param("~K_ds", "0.0"))

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

        self.occupancy_grid = None 


        # self.car_state_sub = rospy.Subscriber("car_state", SpeedDirection, self.carstateCB, queue_size=1)
        # self.timer = rospy.Timer(rospy.Duration(0.4), self.timer_callback)

        self.starting_check_timer = rospy.Timer(rospy.Duration(0.5), self.starting_check_timer)

        self.waypoint_utils = WaypointUtils(
            node=self,
            interpolation_distance=0.1,
            filepath=self.WAYPOINTS_PATH
        )

        self.utils = Utils()

        self.old_path_heading = [None, None]


        # init speed and direction
        self.current_speed = 0.0
        self.current_direction = 0.0
        self.target_velocity = 0.0
        self.old_target_vel = 0.0
        self.old_target_heading = 0.0


        self.obstacle_detected = False

        self.old_error = None
        self.old_crosstrack_error = None

        self.started = False #START CAR NAV

        self.current_pose = None
        self.current_pose_wheelbase_front = None
        self.goal_pos = None
        self.closest_wheelbase_rear_point = None

        self.cells_per_meter = 25

        self.odom_sub = rospy.Subscriber(self.ODOM_TOPIC, Odometry, self.odomCB, queue_size=100)
        self.occup_grid_sub = rospy.Subscriber("ocucpancy_grid", OccupancyGrid, self.occup_grid, queue_size=10)


    def destroy_node(self):
        kill = SpeedDirection()
        kill.speed = 0
        kill.direction = 0
        self.drive_pub.publish(kill)

    def occup_grid(self, msg):
        #Occupancy grid callback.
        #Transform it back into the array it once was
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occupancy_grid = np.ma.array(data, mask=data==-1, fill_value=-1)
        self.grid_height = msg.info.height
        self.stamp = msg.header.stamp
        self.grid_width = msg.info.width

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


    def local_to_grid(self, x,y):
        j = int((x * 25))
        i = int(-y * 25 + ((self.grid_height // 2) - 1))
        i = max(0,i)
        j = max(0,j)
        return (i, j)

    def local_to_grid_parallel(self, x,y):
        j = np.round(x * 25).astype(int)
        i = np.round(-y * 25 + ((self.grid_height // 2) - 1)).astype(int)
        i = np.max(0,i)
        j = np.max(0,j)
        return i, j

    def grid_to_local(self, point):
        i,j = point[0], point[1]
        x = ((self.grid_width - j) - self.grid_width) / -25
        y = (i - ((self.grid_height // 2) - 1)) / 25
        return (x, y)




    ###TAKEN FROM STEVEN GONG

    def check_collision(self, cell_a, cell_b, margin=0):

        t1_start = perf_counter_ns()
        
        offsets = np.array([0, int(-margin*0.25), int(margin*0.25), int(margin*0.5), int(-margin*0.5), int(-margin*0.75), int(margin*0.75), margin, -margin])

        obs, pos = self.traverse_grid_parallel(cell_a, cell_b, offsets)
        
        # print("Elapsed time nick check:",(perf_counter_ns()-t1_start) * 1e-6, 'ms')

        return obs, pos


    def check_for_obstacles(self):

        t1_start = perf_counter_ns()

        
        if self.occupancy_grid is None:
            print("No Occup grid yet")
            return False
        
        path_local = []
        current_pos = np.array(self.local_to_grid(0,0))

        goal_pos = np.array(self.local_to_grid(self.goal_pos[0], -self.goal_pos[1]))

        # print("Curr pos", current_pos)
        # print("Goal pos", goal_pos)
        # print("Goal pos local", self.goal_pos)
    
        # path_local = [self.grid_to_local(current_pos)]
        MARGIN = int(25 * 0.15) # 0.15m margin on each side, since the car is ~0.3m wide


        print("current pos", current_pos)
        # print("goal_pos", goal_pos)

        self.obstacle_detected = False


        obs, new_pos = self.check_collision(current_pos, goal_pos, margin=MARGIN)

        if obs:
            self.obstacle_detected = True
            print("obstacle")
            if not new_pos is None:
                "changed goal pos"
                self.goal_pos = self.grid_to_local(new_pos)
            else: 
                print("but no path")
        else:
            print("No Obs")
            self.obstacle_detected = False

            

        print("self.goal_pos", self.goal_pos)

        # path_local.append(self.goal_pos)

        print("Elapsed time check collision:",(perf_counter_ns()-t1_start) * 1e-6, 'ms')


        if self.obstacle_detected:
            print("OBSTACLE")
            # self.drive_to_target_stanley()
            self.drive_around(self.goal_pos)
        else:
            print("stanley")
            self.drive_to_target_stanley()

        # Visualization
        # self.utils.draw_marker_array("lidar_frame", self.stamp, path_local, self.stanley_avoidance_path_array_pub)
        # self.utils.draw_lines("lidar_frame", self.stamp, path_local, self.stanley_avoidance_path_pub)


    def drive_around(self, point):
        print("DRIVE AROUND")
        #The input is a point in the car's frame of reference. ie if the point is 10,10, it will be 10 m ahead and 10 m to the right of the car. 
        #This makes computing distances and angles trivial
        L = np.linalg.norm(point)
        y = point[1]
        angle = self.K_p_obstacle * (2 * y) / (L**2)
        angle = min(max(math.degrees(angle), -self.STEERING_LIMIT_DEG), self.STEERING_LIMIT_DEG)

        print("angle", angle)

        # determine velocity
        if math.degrees(angle) < 5:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE
        elif math.degrees(angle) < 10.0:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.5
        else:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.3

        self.publish_cmd_vel(velocity, angle)


    def drive_to_target_stanley(self):

        closest_wheelbase_front_point_car, closest_wheelbase_front_point_world = self.waypoint_utils.get_waypoint_stanley(
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



        if path_heading < 0:
            path_heading += 2 * math.pi
        if current_heading < 0:
            current_heading += 2 * math.pi

        if self.old_path_heading[0]:
            traj_yaw_rate = (path_heading - self.old_path_heading[0]) / (rospy.Time.now() - self.old_path_heading[1]).to_sec()
        else:
            traj_yaw_rate = 0.


        if traj_yaw_rate > math.pi:
            traj_yaw_rate -= 2 * math.pi
        elif traj_yaw_rate < -math.pi:
            traj_yaw_rate += 2 * math.pi


        rate_err = traj_yaw_rate - self.curr_ang_vel

        rate_err *= self.K_dh

        rospy.logdebug(f"rate error: {rate_err:.2f}")
        
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
        angle = heading_error + crosstrack_error + rate_err



        diag_msg = Float32MultiArray()
        diag_msg.data = [heading_error, crosstrack_error, rate_err]


        rospy.logdebug(f"heading_error: {heading_error:.2f} ... crosstrack_error: {crosstrack_error:.2f} angle: {np.degrees(angle):.2f}")
        rospy.logdebug(f"current_heading: {current_heading:.2f} ... path_heading: {path_heading:.2f}")

        angle = np.clip(angle, -math.radians(self.STEERING_LIMIT_DEG), math.radians(self.STEERING_LIMIT_DEG))

        velocity = self.target_velocity * self.VELOCITY_PERCENTAGE

        self.publish_cmd_vel(velocity, angle)
        self.diag_pub.publish(diag_msg)


    def publish_cmd_vel(self, velocity, angle):
        if not rospy.is_shutdown():
            drive_msg = SpeedDirection()


            ## If the speed difference is bigger than the brake_threshold, we want to apply the brakes
            # rospy.logwarn((self.target_velocity - self.old_target_vel))
            if ((self.target_velocity - self.old_target_vel)) < -self.BRAKE_THRESHOLD_MS:
                rospy.logwarn("BRAKING BRAKING BRAKING BRAKING BRAKING")
                velocity = velocity * 0.25
                

            #Max Speed is 7 m/s
            velocity_normalized = velocity / 10.0
            velocity_normalized = min(max(velocity_normalized, -1), 1.0)
            drive_msg.speed = velocity_normalized
        
            angle_normalized = angle / np.radians(self.STEERING_LIMIT_DEG)

            drive_msg.direction = -angle_normalized
            self.drive_pub.publish(drive_msg)

    def generate_offset_points(self, start, end, offsets):
        vector = end - start 

        perp_vector  = np.array([-vector[1], vector[0]])

        perp_vector_norm = perp_vector / np.linalg.norm(perp_vector)

        start_points = start + perp_vector_norm * offsets[:,None]
        end_points = np.array(end) + perp_vector_norm * offsets[:, None]
        return start_points, end_points

    def traverse_grid_parallel(self, start, end, offsets, loose=False):
        start = np.array(start)
        end = np.array(end)
        

        if (start < 0).any() or (end < 0).any():
            print("OOB")
            return None, None    

        if loose:
            # print("Trying again")
            offsets = offsets  * 2

        _, ends = self.generate_offset_points(start, end, offsets)

        start_tiled = np.tile(start, (len(offsets), 1))

        vector = end-start

        vectors = ends-start
        
        vector_norm = np.linalg.norm(vector)

        t = np.linspace(0,1, int(vector_norm)+1)

        t = t.reshape(1,-1,1)  

        intermediate = t*vectors[:,None,:]



        points = np.rint((start_tiled[:, None, :] + intermediate).transpose((1,2,0))).astype(int)

        obstacles = []
        for i in range(points.shape[2]):
            ray_points = points[:, :, i]
            valid_mask = (ray_points[:, 0] >= 0) & (ray_points[:, 0] < self.occupancy_grid.shape[0]) & (ray_points[:, 1] >= 0) & (ray_points[:, 1] < self.occupancy_grid.shape[1])
            valid_points = ray_points[valid_mask]
            if(self.occupancy_grid[valid_points[:,0], valid_points[:,1]]).any():
                obstacles.append((True, offsets[i]))
            else:
                obstacles.append((False, offsets[i]))
            

        obstacle_bools = np.array([detection[0] for detection in obstacles])

        filtered_points = points[-1, :, :][:, ~obstacle_bools]

        if np.shape(filtered_points)[1]:
            return obstacles[0][0], filtered_points[:, 0]
        else:
            if not loose:
                return self.traverse_grid_parallel(start, end, offsets, loose=True)
            return obstacles[0][0], None


    
    def odomCB(self, pose_msg: Odometry):

        # print("Elapsed time ODOM:",(perf_counter_ns()-self.t1_start) * 1e-6, 'ms') 
        self.t1_start = perf_counter_ns()


        self.current_pose = pose_msg.pose.pose
        self.curr_ang_vel = pose_msg.twist.twist.angular.z
        
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

        self.closest_wheelbase_rear_point, self.target_velocity = self.waypoint_utils.get_closest_waypoint_with_velocity(
            self.current_pose
        )

        self.utils.draw_marker(
            pose_msg.header.frame_id,
            pose_msg.header.stamp,
            self.closest_wheelbase_rear_point,
            self.current_waypoint_pub,
            color="blue",
        )



        self.goal_pos, goal_pos_world = self.waypoint_utils.get_waypoint(self.current_pose, self.target_velocity)



        # print(self.goal_pos)
        # print("gp world", goal_pos_world)


        self.utils.draw_marker(pose_msg.header.frame_id, pose_msg.header.stamp, goal_pos_world, self.waypoint_pub, color="red")


        

        if self.started:
            # self.drive_to_target_stanley()
            self.check_for_obstacles()


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
        self.node = node

        self.waypoints_world, self.velocities = self.load_and_interpolate_waypoints(
            file_path=filepath, interpolation_distance=interpolation_distance
        )

        self.index = 0 
        self.velocity_index = 0
        self.heading_index = 0

        self.min_lookahead = 1.5
        self.max_lookahead = 3.0

        self.min_lookahead_speed = 3.0
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

        return self.waypoints_world[self.velocity_index], self.velocities[self.velocity_index]

    def get_waypoint(self, pose, target_velocity, fixed_lookahead=None):
        # get current position of car
        if pose is None:
            return
        position = (pose.position.x, pose.position.y, 0)

        # transform way-points from world to vehicle frame of reference
        
        waypoints_car = self.transform_waypoints(self.waypoints_world, position, pose)
       
        # get distance from car to all waypoints
        distances = np.linalg.norm(waypoints_car, axis=1)

        # get indices of waypoints that are within L, sorted by descending distance
        # Use dynamic lookahead for this part

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

        indices_L = np.argsort(np.where(distances < self.L, distances, -1))[::-1]

        # print(self.L)

        # set goal point to be the farthest valid waypoint within distance L
        for i in indices_L:
            # check waypoint is in front of car
            x = waypoints_car[i][0]
            if x > 0:
                self.index = i
                return waypoints_car[self.index], self.waypoints_world[self.index]
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

        return waypoints_car[index], self.waypoints_world[index]


    def load_and_interpolate_waypoints(self, file_path, interpolation_distance=0.2):
        # Read waypoints from csv, first two columns are x and y, third column is velocity
        # Exclude last row, because that closes the loop
        points = np.genfromtxt(file_path, delimiter=",")[:, :2]
        velocities = np.genfromtxt(file_path, delimiter=",")[:, 2]
        # headings = np.genfromtxt(file_path, delimiter=",")[:, 3]


        # Add first point as last point to complete loop
        rospy.loginfo("Velocities: " + str(velocities))
        # rospy.loginfo("Headings: " + str(headings))

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

            # heading_interpolator = interp1d(distance, headings, kind="slinear")
            # interpolated_headings = heading_interpolator(alpha)

            # Add z-coordinate to be 0
            interpolated_points = np.hstack((interpolated_points, np.zeros((interpolated_points.shape[0], 1))))
            assert len(interpolated_points) == len(interpolated_velocities)
            # assert len(interpolated_headings) == len(interpolated_velocities)
            return interpolated_points, interpolated_velocities

        else:
            # Add z-coordinate to be 0
            points = np.hstack((points, np.zeros((points.shape[0], 1))))
            return points, velocities


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