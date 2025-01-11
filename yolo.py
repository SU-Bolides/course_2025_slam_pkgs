from ultralytics import YOLO
import cv2

# Chargement du modèle YOLO pré-entraîné
model = YOLO("yolov8n.pt")  # modèle YOLO généraliste

# Accéder à la webcam
cap = cv2.VideoCapture(0)  # 0 pour la webcam par défaut

if not cap.isOpened():
    print("Erreur : Impossible d'accéder à la webcam.")
    exit()

print("Appuyez sur 'q' pour quitter.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Erreur : Impossible de lire une image depuis la webcam.")
        break

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    results = model(frame_rgb)

    # Annotation des objets détectés
    for result in results[0].boxes.data.tolist():
        x1, y1, x2, y2, score, class_id = result
        if score > 0.5:  # Seuil de confiance

            # Calculer la taille de la boîte (aire)
            width = x2 - x1
            height = y2 - y1
            area = width * height

            # Seuil pour distinguer les objets proches
            threshold_area = 0.3 * (frame.shape[0] * frame.shape[1])  # 30% de la taille de l'image
            label = f"{model.names[int(class_id)]} {score:.2f}"

            # Choix de la couleur selon la proximité
            color = (0, 0, 255) if area > threshold_area else (0, 255, 0)  # Rouge si proche, Vert sinon

            # Affichage du rectangle et de son her l'étiquette
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, label, (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Afficher l'image annotée en direct
    cv2.imshow("Détection d'objets avec YOLOv8", frame)

    # Quitter en appuyant sur 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources
cap.release()
cv2.destroyAllWindows()

