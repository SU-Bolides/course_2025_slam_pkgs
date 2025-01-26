import cv2
import numpy as np
import os

# Paramètres de l'échiquier
CHECKERBOARD = (6, 5)  
SQUARE_SIZE = 0.024  # Taille du coté d'un carré (2.4 cm = 0.024 m)

# Préparation des points 3D dans le monde réel 
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

# Listes pour stocker les points du monde réel et les points de l'image
objpoints = []  # Points 3D dans le monde réel
imgpoints = []  # Points 2D dans l'image

# Dossier pour sauvegarder les captures
capture_folder = "calibration_captures"
os.makedirs(capture_folder, exist_ok=True)

# Fonction pour effectuer la calibration de la caméra
def calibrate_camera():
    global objpoints, imgpoints
    print("Appuyez sur 's' pour capturer une image ou 'q' pour quitter.")
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Erreur : Impossible d'accéder à la caméra.")
        return None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erreur : Impossible de lire l'image depuis la caméra.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
            cv2.drawChessboardCorners(frame, CHECKERBOARD, corners, ret)

        cv2.imshow("Capture d'échiquier", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and ret: 
            img_name = os.path.join(capture_folder, f"capture_{len(objpoints)}.jpg")
            cv2.imwrite(img_name, frame)
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"Image capturée et sauvegardée : {img_name}")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(objpoints) > 0:
        print("Calibration de la caméra en cours...")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )
        print("Matrice intrinsèque :\n", camera_matrix)
        print("Coefficients de distorsion :\n", dist_coeffs)
        return camera_matrix, dist_coeffs
    else:
        print("Aucune image valide capturée pour la calibration.")
        return None
