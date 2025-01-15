import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt
import os
from pathlib import Path

def getTrack(filename):
    track_file = os.path.join(str(Path(__file__).parent), filename)
    array = np.loadtxt(track_file, delimiter=",")
    return array

### Apply Gaussian Smoothing ###
def smooth_track(track_points, sigma=2):
    x_smooth = gaussian_filter1d(track_points[:, 0], sigma=sigma)
    y_smooth = gaussian_filter1d(track_points[:, 1], sigma=sigma)
    return np.column_stack((x_smooth, y_smooth))

### Generate Spline Representation of Track ###
def generate_track_spline(track_points):
    s = np.zeros(track_points.shape[0])
    for i in range(1, len(track_points)):
        s[i] = s[i-1] + np.linalg.norm(track_points[i] - track_points[i-1])
    
    x_spline = CubicSpline(s, track_points[:, 0])
    y_spline = CubicSpline(s, track_points[:, 1])
    
    return s, x_spline, y_spline

### Compute Curvature from Splines ###
def compute_curvature(s, x_spline, y_spline):
    x_s_prime = x_spline.derivative()
    y_s_prime = y_spline.derivative()
    x_s_double_prime = x_s_prime.derivative()
    y_s_double_prime = y_s_prime.derivative()
    
    psi = np.arctan2(y_s_prime(s), x_s_prime(s))
    kappa = (x_s_prime(s) * y_s_double_prime(s) - y_s_prime(s) * x_s_double_prime(s)) / (x_s_prime(s)**2 + y_s_prime(s)**2)**(3/2)
    
    return kappa, psi

def plot_spline_track(
    s, x_spline, y_spline, heading_spline, curvature_spline,
    x_actual=None, y_actual=None
):
    """
    Plots:
      - The spline track in XY-plane
      - (Optionally) the actual track in XY-plane for comparison
      - Heading vs. arc length
      - Curvature vs. arc length

    Args:
        s (array-like): Arc length array, e.g. [0, 0.1, 0.2, ..., s_max].
        x_spline, y_spline (array-like): Centerline (x, y) coordinates from spline.
        heading_spline (array-like): Heading (orientation) along the spline (rad or deg).
        curvature_spline (array-like): Curvature along the spline.
        x_actual, y_actual (array-like, optional): Actual track points (x, y) if you have them.
    """

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # 1) XY-plane
    axes[0].plot(x_spline, y_spline, 'b-o', label="Spline Track")
    if x_actual is not None and y_actual is not None:
        axes[0].plot(x_actual, y_actual, 'r--', label="Actual Track")
    axes[0].set_aspect('equal', adjustable='datalim')
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].set_title("Track in XY-plane")
    axes[0].legend()

    # 2) Heading vs. s
    axes[1].plot(s, heading_spline, 'g-o')
    axes[1].set_xlabel("Arc length s [m]")
    axes[1].set_ylabel("Heading [rad or deg]")
    axes[1].set_title("Heading vs. s")
    axes[1].grid(True)

    # 3) Curvature vs. s
    axes[2].plot(s, curvature_spline, 'm-o')
    axes[2].set_xlabel("Arc length s [m]")
    axes[2].set_ylabel("Curvature [1/m]")
    axes[2].set_title("Curvature vs. s")
    axes[2].grid(True)

    plt.tight_layout()
    plt.show()

# Load track and apply smoothing
track = getTrack("su_bigmap.txt")
track = np.vstack([track, track[0]])  # Close the loop
track = track[::4, :]  # Downsample
track_smoothed = smooth_track(track, sigma=2)  # Apply Gaussian smoothing

# Generate spline from smoothed track
s, x_s, y_s = generate_track_spline(track_smoothed)
kappa, psi = compute_curvature(s, x_s, y_s)

# Save processed data
data = np.array([s, track_smoothed[:, 0], track_smoothed[:, 1], psi, kappa]).transpose()
np.savetxt("su_bigmap_splined.txt", data, fmt='%0.4f', delimiter=',')


plot_spline_track(*data.T, track[:, 0], track[:, 1])


# Plot results
plt.figure()
plt.plot(track[:, 0], track[:, 1], 'o-', label="Original Track", alpha=0.5)
plt.plot(track_smoothed[:, 0], track_smoothed[:, 1], 's-', label="Smoothed Track")
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Track Representation")
plt.legend()
plt.grid()
plt.show()
