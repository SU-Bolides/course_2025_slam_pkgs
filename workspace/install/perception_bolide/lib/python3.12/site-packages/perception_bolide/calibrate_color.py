#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import rclpy
from rclpy.node import Node

# Import ROS 2 message types
from sensor_msgs.msg import Image as SensorImage
from std_msgs.msg import Bool

# Auteur
__author__ = "Raphael KHORASSANI"
__status__ = "Development"


class DetectColor(Node):
    def __init__(self):
        super().__init__("calibrate_color")

        # Publisher pour indiquer la fin de la calibration
        self.pub = self.create_publisher(Bool, "/is_auto_calibration_done", 10)

        # Subscriber pour déclencher la calibration
        self.create_subscription(Bool, "/do_an_auto_calibration", self.listener, 10)

        self.subscriber = None  # Pour stocker l'abonnement à l'image
        self.color = "no_one"

    def listener(self, msg):
        """Callback activé lorsqu'on demande une auto-calibration"""
        if msg.data:  # Vérifier si la demande de calibration est vraie
            self.color = self.get_parameter_or("/color_to_calibrate", "no_one")

            if self.color != "no_one":
                # S'abonner au topic d'image pour la calibration
                self.subscriber = self.create_subscription(
                    SensorImage, "/raw_image_data", self.callback_image, 10
                )
            else:
                self.get_logger().info("Aucune couleur à calibrer.")

    def callback_image(self, image_data):
        """Callback pour traiter une image et extraire la couleur dominante"""
        h, w = image_data.height, image_data.width
        im = np.frombuffer(image_data.data, dtype=np.uint8).reshape((h, w, 3))

        # Définition de la zone d'intérêt
        H, W, _ = im.shape
        wH, wW = int(H * 0.2), int(W * 0.4)
        zone_of_interest = im[(W - wW) // 2 : (W - wW) // 2 + wW, (H - wH) // 2 : (H - wH) // 2 + wH]

        # Calcul de la médiane des couleurs
        values = np.median(zone_of_interest, axis=(0, 1)).astype(np.uint8).tolist()

        # Définition du paramètre de couleur calibrée
        param_name = f"/{self.color}_RGB"
        self.set_parameters([rclpy.parameter.Parameter(param_name, rclpy.Parameter.Type.INTEGER_ARRAY, values)])

        # Réinitialiser le paramètre de calibration
        self.set_parameters([rclpy.parameter.Parameter("/color_to_calibrate", rclpy.Parameter.Type.STRING, "no_one")])

        self.get_logger().info(f"Calibration terminée pour {self.color}")

        # Publier un message pour indiquer que la calibration est finie
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)

        # Désactiver l'abonnement à l'image après calibration
        self.destroy_subscription(self.subscriber)


def main(args=None):
    rclpy.init(args=args)
    node = DetectColor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
