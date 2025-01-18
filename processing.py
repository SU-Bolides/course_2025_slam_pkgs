import numpy as np
from scipy import signal
import math
from sklearn.cluster import DBSCAN
import pickle

# Constantes
IS_OCCUPIED = 100  # Valeur pour une cellule occupée dans la grille d'occupation
resolution = 0.04  # Résolution de la carte (en mètres par pixel)
CELLS_PER_METER = 1 / resolution  # Nombre de cellules par mètre, inverse de la résolution

# Fonction pour créer une grille d'occupation basée sur les données du LiDAR
def lidar_to_grid(angles, distances, grid_height, grid_width, robot_pos, cpm = CELLS_PER_METER):
    """
    Transforme les données brutes du LiDAR en une grille d'occupation.
    - `angles` : angles mesurés par le LiDAR.
    - `distances` : distances associées à chaque angle.
    - `grid_height`, `grid_width` : dimensions de la grille.
    - `robot_pos` : position du robot dans le référentiel monde.

    Retourne : la grille d'occupation, la position du robot dans la grille,
    et les coordonnées des points LiDAR (x, y).
    """
    occupancy_grid = np.zeros((grid_height, grid_width), dtype=int)  # Initialisation de la grille
    x = distances * np.cos(angles)  # Coordonnées X des points LiDAR
    y = distances * np.sin(angles)  # Coordonnées Y des points LiDAR

    # Transformation des coordonnées LiDAR en indices de la grille
    i = np.round((x * -cpm) + (grid_height - 1)).astype(int)
    j = np.round((y * -cpm) + ((grid_width // 2) - 1)).astype(int)
    robot_pos_grid = (  # Position du robot dans la grille d'occupation
        np.round((robot_pos[0] * -cpm) + (grid_height - 1)).astype(int),
        np.round((robot_pos[1] * -cpm) + (grid_width // 2) - 1).astype(int)
    )

    # Filtrage des points hors limites de la grille
    valid_indices = (i >= 0) & (i < grid_height) & (j >= 0) & (j < grid_width)
    occupancy_grid[i[valid_indices], j[valid_indices]] = IS_OCCUPIED  # Marque les cellules occupées

    # Applique un noyau pour élargir les cellules occupées (sorte de dilatation)
    kernel = np.ones(shape=[2, 2])
    occupancy_grid = signal.convolve2d(
        occupancy_grid.astype("int"), kernel.astype("int"), boundary="symm", mode="same"
    )
    # Limite les valeurs entre -1 et 100
    occupancy_grid = np.clip(occupancy_grid, -1, IS_OCCUPIED)
    return occupancy_grid, robot_pos_grid, (x, y)


# Fonction pour détecter les clusters d'obstacles dans la grille d'occupation
def detect_clusters(occupancy_grid, eps=3, min_samples=10):
    """
    Détecte des clusters d'obstacles dans une grille d'occupation en utilisant DBSCAN.
    - `occupancy_grid` : grille d'occupation.
    - `eps` : distance maximale entre deux points pour les considérer comme dans le même cluster.
    - `min_samples` : nombre minimal de points pour former un cluster.

    Retourne : une liste des centres des clusters, et une grille d'occupation mise à jour.
    """
    # Réduction du bruit avec un filtre médian
    occupancy_grid_filtered = signal.medfilt2d(occupancy_grid, 3)
    # Trouve les points occupés
    points = np.argwhere(occupancy_grid_filtered == IS_OCCUPIED)
    # Applique l'algorithme DBSCAN
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    # Récupère les étiquettes des clusters
    labels = clustering.labels_
    core_samples_mask = np.zeros_like(labels, dtype=bool)
    core_samples_mask[clustering.core_sample_indices_] = True
    clusters = []
    for label in set(labels):
        if label != -1:  # Ignore les points considérés comme du bruit
            class_member_mask = labels == label
            xy = points[class_member_mask & core_samples_mask]
            if len(xy) > 0 and len(xy) < 30:  # Filtre les clusters trop petits ou trop grands
                x = round(np.mean(xy[:, 0]))
                y = round(np.mean(xy[:, 1]))
                clusters.append([x, y])
    return clusters


# Convertit les coordonnées de la grille en coordonnées relatives au robot
def grid_to_rel(grid_point, cpm, grid_shape):
    """
    Convertit un point de la grille en coordonnées relatives au robot en mettre.
    - `grid_point` : point de la grille.
    - `cpm` : cellules par mètre.
    - `grid_shape` : dimensions de la grille.

    Retourne : coordonnées (x, y) relatives.
    """
    x_rel = ((grid_shape[0] - 1) - grid_point[0]) / cpm
    y_rel = ((grid_shape[1] // 2) - 1 - grid_point[1]) / cpm
    return x_rel, y_rel


# Convertit les coordonnées relatives au robot en coordonnées globales
def rel_to_world(x_rel, y_rel, theta, robot_pos):
    """
    Transforme des coordonnées relatives au robot en coordonnées monde.
    - `x_rel`, `y_rel` : coordonnées relatives.
    - `theta` : orientation du robot.
    - `robot_pos` : position monde du robot.

    Retourne : coordonnées monde (x, y).
    """
    x_world = robot_pos[0] + (math.cos(theta) * x_rel - math.sin(theta) * y_rel)
    y_world = robot_pos[1] + (math.sin(theta) * x_rel + math.cos(theta) * y_rel)
    return x_world, y_world


# Calcule l'angle d'orientation (theta) à partir d'un quaternion
def calculate_theta(orientation):
    """
    Calcule l'orientation du robot (theta) à partir de son quaternion.
    - `orientation` : quaternion (x, y, z, w).

    Retourne : angle theta, rotation autour de z.
    """
    x, y, z, w = orientation
    theta = math.atan2(2 * (x * y + z * w), 1 - 2 * (y ** 2 + z ** 2))
    return theta


# Transforme les coordonnées locales en coordonnées de la carte
def world_to_map_grid(map_origin, points, map_height, map_resolution=0.04):
    """
    Convertit des coordonnées locales en pixels de la carte.
    - `map_origin` : origine de la carte en coordonnées monde.
    - `points` : coordonnées monde (x, y).
    - `map_height` : hauteur de la carte.
    - `map_resolution` : résolution de la carte.

    Retourne : coordonnées (px, py) dans la carte en pixel.
    """
    px = map_height - int(np.round((points[1] - map_origin[1]) / map_resolution))
    py = int(np.round((points[0] - map_origin[0]) / map_resolution))
    return px, py

def load_dico(nom_fichier_sauvegarde):
    """
    Charge un dictionnaire depuis un fichier pickle.
    """
    with open(nom_fichier_sauvegarde, 'rb') as fichier:
        return pickle.load(fichier)
