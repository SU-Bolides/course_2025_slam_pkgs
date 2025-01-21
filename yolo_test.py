from ultralytics import YOLO
import cv2
import numpy as np
import os
import time 

# Charger les paramètres de calibration à partir des images existantes
def calibrate_from_existing_images():
    CHECKERBOARD = (6, 5)  # Dimensions de l'échiquier
    SQUARE_SIZE = 0.024  # Taille réelle d'une case en mètres

    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2) * SQUARE_SIZE

    objpoints = []  # Points 3D dans le monde réel
    imgpoints = []  # Points 2D dans l'image

    capture_folder = "calibration_captures"
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

# Charger les paramètres de calibration
camera_matrix, dist_coeffs = calibrate_from_existing_images()
if camera_matrix is None or dist_coeffs is None:
    print("Erreur : Calibration échouée. Assurez-vous que les images d'échiquier sont valides.")
    exit()

# Charger le modèle YOLOv8 ultra-léger
model = YOLO("yolov8n.pt")  # YOLOv8 Nano pour la rapidité
model.overrides["device"] = "cpu"  # Force l'utilisation du CPU

KNOWN_WIDTH = 0.5  # Largeur supposée moyenne d'un objet

# Fonction pour calculer les coordonnées 3D
def calculate_xyz(bbox, camera_matrix, known_width):
    x1, y1, x2, y2 = bbox
    pixel_width = abs(x2 - x1)
    focal_length = camera_matrix[0, 0]
    z = (focal_length * known_width) / pixel_width
    u = (x1 + x2) / 2
    v = (y1 + y2) / 2
    x = (u - camera_matrix[0, 2]) * z / focal_length
    y = (v - camera_matrix[1, 2]) * z / camera_matrix[1, 1]
    return x, y, z

# Fonction pour guider la voiture
def guide_vehicle(objects):
    for obj in objects:
        label, x, y, z = obj
        if z < 1.0:  # Zone critique
            print(f"Obstacle proche ({label}) ! Freinage nécessaire.")
        elif z < 3.0:  # Zone de pré-alerte
            if x < 0:  # Obstacle à gauche
                print(f"Obstacle à gauche ({label}) ! Tourner à droite.")
            elif x > 0:  # Obstacle à droite
                print(f"Obstacle à droite ({label}) ! Tourner à gauche.")
        else:
            print(f"Zone sûre pour ({label}). Continuer tout droit.")

# Initialiser la webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Erreur : Impossible d'accéder à la webcam.")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur : Impossible de lire l'image depuis la webcam.")
        break

    start_time = time.time()

    results = model.predict(source=frame, imgsz=320, conf=0.5, verbose=False)

    detected_objects = []
    for result in results:
        for box, cls in zip(result.boxes.xyxy, result.boxes.cls):
            x1, y1, x2, y2 = map(int, box)
            class_id = int(cls)
            label = model.names[class_id]

            x_real, y_real, z_real = calculate_xyz((x1, y1, x2, y2), camera_matrix, KNOWN_WIDTH)
            detected_objects.append((label, x_real, y_real, z_real))

            info = f"{label}, x: {x_real:.2f}m, y: {y_real:.2f}m, z: {z_real:.2f}m"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, info, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    guide_vehicle(detected_objects)

    end_time = time.time()
    inference_time = end_time - start_time
    cv2.putText(frame, f"Inference Time: {inference_time:.3f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Optimized Real-time Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
