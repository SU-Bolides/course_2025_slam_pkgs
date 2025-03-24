import cv2
import numpy as np
from processing_dbscan import *
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

SAMPLES = 10 # nombre minimal de points pour former un cluster.
EPS = 3 # distance maximale entre deux points pour les considérer comme dans le même cluster.

class DBSCAN(Node):
    def __init__(self):
        super().__init__('dbscan_node')
        self.get_logger.info("[INFO] -- Initializing the dbscan data node")
        
        # SUBSCRIBER
        self.sub = self.create_subscription(OccupancyGrid, '/occupancy_grid', self.occupCB, 10)
        # PUBLISHER
        self.obstacle_pub = self.create_publisher(Marker, '/obstacle_marker', 10)
        
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
        
    def occupCB(self, msg:OccupancyGrid):
        data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occupancy_grid = np.ma.array(data, mask=data==-1, fill_value=-1)
        self.grid_height = msg.info.width
        self.stamp = msg.header.stamp
        self.grid_width = msg.info.height
        self.CELL_Y_OFFSET = (self.grid_width // 2) - 1


        clusters = detect_clusters(self.occupancy_grid)  # Identification des clusters d'obstacles à l'aide de la fonction detect_clusters

        # Parcours de chaque obstacle détecté (clusters contient les positions des obstacles)
        for obstacle in clusters:
            # Marquage de chaque obstacle sur la grille copiée (occ_with_car)
            print(obstacle)

            self.draw_marker(
                "lidar_frame",
                self.get_clock().now().to_msg(),
                [obstacle[0], obstacle[1]],
                self.obstacle_pub,
                color="blue",
            )

def main(args=None):
    rclpy.init(args=args)
    
    CProcess = DBSCAN()
