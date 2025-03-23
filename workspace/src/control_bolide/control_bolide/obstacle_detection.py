#!/usr/bin/env python3

__author__ = "Nicolas HAMMJE"
__status__ = "In Developpement"

#%% IMPORTS
import rclpy
from rclpy.node import Node
import math
from std_msgs.msg import Float32MultiArray
from control_bolide.msg import SpeedDirection
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from scipy.interpolate import interp1d
# from scipy import signal

from inputimeout import inputimeout, TimeoutOccurred
import copy



#%% CLASS
class ObstacleDetection:
    '''
    This class implements a stanley controller and (soon) an obstacle avoidance system. 
    '''
    def __init__(self):
        rclpy.init()
        self.declare_parameter("K_p_obstacle", "0.5")
        self.declare_parameter("velocity_percentage", "0.3")
        self.declare_parameter("steering_limit_deg", "15.2")
        self.declare_parameter("brake_threshold_ms", "0.5")



        self.K_p_obstacle = float(self.get_parameter("K_p_obstacle").get_parameter_value().double_value)
        self.VELOCITY_PERCENTAGE = float(self.get_parameter("velocity_percentage").get_parameter_value().double_value)
        self.STEERING_LIMIT_DEG = float(self.get_parameter("steering_limit_deg").get_parameter_value().double_value)
        self.BRAKE_THRESHOLD_MS = float(self.get_parameter("brake_threshold_ms").get_parameter_value().double_value)

        self.occup_grid_sub = self.create_subscription(OccupancyGrid, "ocucpancy_grid", self.occup_grid, 1)
        self.current_waypoint_pub = self.create_publisher(Marker, "move_around", 10)
        self.waypoint_pub = self.create_publisher(Marker, "next_waypoint", 10)


        self.occupancy_grid = None 

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

        self.grid_height = 0 
        self.grid_width = 0
        rclpy.spin()

    def destroy_node(self):
        kill = SpeedDirection()
        kill.speed = 0
        kill.direction = 0

    def occup_grid(self, msg):
        #Occupancy grid callback.
        #Transform it back into the array it once was
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.width, msg.info.height)
        self.occupancy_grid = np.ma.array(data, mask=data==-1, fill_value=-1)
        self.grid_height = msg.info.width
        self.stamp = msg.header.stamp
        self.grid_width = msg.info.height
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1
        self.check_for_obstacles()

    ###TAKEN FROM STEVEN GONG

    def check_collision(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        One of the issues is that if the starting cell is next to a wall, then it already considers there to be a collision.
        See check_collision_loose


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (cell_a[0], cell_a[1] + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == 100:
                        return True
                except:
                    self.get_logger().info(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    def check_collision_loose(self, cell_a, cell_b, margin=0):
        """
        Checks whether the path between two cells
        in the occupancy grid is collision free.

        The margin is done by checking if adjacent cells are also free.

        This looser implementation only checks half way for meeting the margin requirement.


        Args:
            cell_a (i, j): index of cell a in occupancy grid
            cell_b (i, j): index of cell b in occupancy grid
            margin (int): margin of safety around the path
        Returns:
            collision (bool): whether path between two cells would cause collision
        """
        for i in range(-margin, margin + 1):  # for the margin, check
            cell_a_margin = (int((cell_a[0] + cell_b[0]) / 2), int((cell_a[1] + cell_b[1]) / 2) + i)
            cell_b_margin = (cell_b[0], cell_b[1] + i)
            for cell in self.utils.traverse_grid(cell_a_margin, cell_b_margin):
                if (cell[0] * cell[1] < 0) or (cell[0] >= self.grid_height) or (cell[1] >= self.grid_width):
                    continue
                try:
                    if self.occupancy_grid[cell] == 100:
                        return True
                except:
                    self.get_logger().info(f"Sampled point is out of bounds: {cell}")
                    return True
        return False

    
    def local_to_grid(self, x, y):
        i = int(x * -25 + (self.grid_height - 1))
        j = int(y * -25 + self.CELL_Y_OFFSET)
        return (i, j)

    def local_to_grid_parallel(self, x, y):
        i = np.round(x * -25 + (self.grid_height - 1)).astype(int)
        j = np.round(y * -25 + self.CELL_Y_OFFSET).astype(int)
        return i, j

    def grid_to_local(self, point):
        i, j = point[0], point[1]
        x = (i - (self.grid_height - 1)) / -25
        y = (j - self.CELL_Y_OFFSET) / -25
        return (x, y)




    def check_for_obstacles(self):

        if self.occupancy_grid is None:
            return False
        
        path_local = []
        current_pos = np.array(self.local_to_grid(0,0))
        goal_pos = np.array(self.local_to_grid(0.5, 0))
        target = None
        path_local = [self.grid_to_local(current_pos)]
        MARGIN = int(25 * 0.15) # 0.15m margin on each side, since the car is ~0.3m wide


        if self.check_collision(current_pos, goal_pos, margin=MARGIN):
            self.obstacle_detected = True

            print("OBSTACLE", current_pos, goal_pos)


            shifts = [i * (-1)**i for i in range(1, 21)]

            found = False
            for shift in shifts:
                # We consider various points to the left and right of the goal position
                new_goal = goal_pos + np.array([0, shift])

                # If we are currently super close to the wall, this logic doesn't work
                if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                    target = self.grid_to_local(new_goal)
                    found = True
                    path_local.append(target)
                    self.get_logger().info("Found condition 1")
                    break

            if not found:
                # This means that the obstacle is very close to us, we need even steeper turns
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision(current_pos, new_goal, margin=int(1.5 * MARGIN)):
                        target = self.grid_to_local(new_goal)
                        found = True
                        path_local.append(target)
                        self.get_logger().info("Found condition 2")
                        break

            if not found:
                # Try again with a looser collision checker, we are probably very close to the obstacle, so check only collision free in the second half
                middle_grid_point = np.array(current_pos + (goal_pos - current_pos) / 2).astype(int)

                for shift in shifts:
                    new_goal = middle_grid_point + np.array([0, shift])
                    if not self.check_collision_loose(current_pos, new_goal, margin=MARGIN):
                        target = self.grid_to_local(new_goal)
                        found = True
                        path_local.append(target)
                        self.get_logger().info("Found condition 3")
                        break

        else:
            self.obstacle_detected = False
            target = self.grid_to_local(goal_pos)
            path_local.append(target)

        self.utils.draw_marker("lidar_frame", self.stamp, target, self.waypoint_pub, color="red")
        self.utils.draw_marker("lidar_frame", self.stamp, self.grid_to_local(goal_pos), self.current_waypoint_pub, color="green")
        if target:
            if self.obstacle_detected:
                print("OBSTACLE")
                self.drive_around(target)
            else:
                self.drive_to_target_stanley()
        else:
            self.get_logger().info("Allah take the wheel IDK what to do")
            self.drive_to_target_stanley()



    def drive_around(self, point):
        #The input is a point in the car's frame of reference. ie if the point is 10,10, it will be 10 m ahead and 10 m to the right of the car. 
        #This makes computing distances and angles trivial
        L = np.linalg.norm(point)
        y = point[1]
        angle = self.K_p_obstacle * (2 * y) / (L**2)
        angle = min(max(math.degrees(angle), -self.STEERING_LIMIT_DEG), self.STEERING_LIMIT_DEG)

        # determine velocity
        if math.degrees(angle) < 5:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE
        elif math.degrees(angle) < 10.0:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.3
        else:
            velocity = self.target_velocity * self.VELOCITY_PERCENTAGE * 0.1


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
            marker.lifetime = Duration(seconds=0.1).to_msg()
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


#%% MAIN
if __name__ == '__main__':
    print("Stanley Avoidance Initialized")
    controller = ObstacleDetection()
    quit()