from ultralytics import YOLO
import cv2
import time
import numpy as np
from calibration import calibrate_camera

# Chargement du modèle YOLOv8
model = YOLO("yolov8n.pt")  # réduction du temps d'inférence par le modèle nano 

# Initialisation de la webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Erreur : Impossible d'accéder à la webcam.")
    exit()


while True:
    # Capture d'une image de la webcam
    ret, frame = cap.read()
    if not ret:
        print("Erreur : Impossible de lire l'image depuis la webcam.")
        break

    # Détection et mesure du temps d'inférence
    start_time = time.time()
    results = model.predict(source=frame, imgsz=320, conf=0.5, show=False)  # Réduire imgsz pour plus de rapidité
    end_time = time.time()

    inference_time = end_time - start_time
    print(f"Temps d'inférence : {inference_time:.3f} secondes")

    # Annotation des objets détectés
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, confidence, class_id = map(float, result[:6])
        label = f"{model.names[int(class_id)]} {confidence:.2f}"

        # Calcul du centre de la boîte englobante
        center_x = (x1 + x2) / 2
        center_y = (y1 + y2) / 2

        # Transformation en coordonnées locales
        # print(f"Objet: {label}, Coordonnées locales (x, y, z) : {local_coords}")

        # Dessin des boîtes englobantes
        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, "OBSTACLE, A EVITER", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Affichage du temps d'inférence
    cv2.putText(frame, f"Temps d'inference: {inference_time:.3f}s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    # Affichage de l'image avec les détections
    cv2.imshow("YOLOv8 Object Detection", frame)

    # Quitter avec la touche 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()
