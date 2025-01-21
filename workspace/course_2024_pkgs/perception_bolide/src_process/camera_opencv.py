#!/usr/bin/env python3

from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import numpy as np
import time
from ultralytics import YOLO
import os
import cv2

class CameraProcess:

    def __init__(self):
        
        
        rospy.loginfo("[INFO] -- Initializing the camera process data node")
        rospy.init_node('camera_process')

        self.bridge = CvBridge()

        self.cv_image = None

        self.model = YOLO("yolov8n.pt")  # YOLOv8 Nano pour la rapidité
        # self.model.overrides["device"] = "cpu"  # Force l'utilisation du CPU

        self.camera_matrix, self.dist_coeffs = self.calibrate_from_existing_images()
        if self.camera_matrix is None or self.dist_coeffs is None:
            print("Erreur : Calibration échouée. Assurez-vous que les images d'échiquier sont valides.")
            exit()

        rospy.Subscriber("/raw_image_data", Image, self.callback_image)

        rospy.spin()

    # Charger les paramètres de calibration à partir des images existantes
    def calibrate_from_existing_images(self):
        CHECKERBOARD = (6, 5)  # Dimensions de l'échiquier
        SQUARE_SIZE = 0.024  # Taille réelle d'une case en mètres

        objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

        objpoints = []  # Points 3D dans le monde réel
        imgpoints = []  # Points 2D dans l'image

        capture_folder = "/home/john/robot_ws/src/course_2024_pkgs/perception_bolide/src_process/calibration_captures"
        images = [os.path.join(capture_folder, f) for f in os.listdir(capture_folder) if f.endswith(".jpg")]

        if not images:
            print("Aucune image trouvée dans le dossier 'calibration_captures'.")
            return None, None

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
            if ret:
                objpoints.append(objp)
                imgpoints.append(corners)

        if len(objpoints) > 0:
            print("Calibration de la caméra en cours...")
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
            print("Matrice intrinsèque :\n", camera_matrix)
            print("Coefficients de distorsion :\n", dist_coeffs)
            return camera_matrix, dist_coeffs
        else:
            print("Impossible de calibrer la caméra. Vérifiez les images d'échiquier.")
            return None, None


    def callback_image(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.process_image(self.cv_image)

    def calculate_xyz(self, bbox, camera_matrix, known_width=0.5):
        x1, y1, x2, y2 = bbox
        pixel_width = abs(x2 - x1)
        focal_length = camera_matrix[0, 0]
        z = (focal_length * known_width) / pixel_width
        u = (x1 + x2) / 2
        v = (y1 + y2) / 2
        x = (u - camera_matrix[0, 2]) * z / focal_length
        y = (v - camera_matrix[1, 2]) * z / camera_matrix[1, 1]
        return x, y, z

    def process_image(self, image):

        self.image = np.copy(image)

        start_time = time.time()

        results = self.model.predict(source=self.image, conf=0.5, verbose=False)
        detected_objects = []
        for result in results:
            for box, cls in zip(result.boxes.xyxy, result.boxes.cls):
                x1, y1, x2, y2 = map(int, box)
                class_id = int(cls)
                label = self.model.names[class_id]

                x_real, y_real, z_real = self.calculate_xyz((x1, y1, x2, y2), self.camera_matrix)
                detected_objects.append((label, x_real, y_real, z_real))

                info = f"{label}, x: {x_real:.2f}m, y: {y_real:.2f}m, z: {z_real:.2f}m"
                print(info)
                # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                # cv2.putText(frame, info, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        end_time = time.time()
        inference_time = end_time - start_time  
        print(f"Inference Time: {inference_time:.3f}s")

if __name__ == '__main__':
    
    CProcess = CameraProcess()
