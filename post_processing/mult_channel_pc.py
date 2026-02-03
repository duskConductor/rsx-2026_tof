import pandas as pd
import numpy as np
import time
import open3d as o3d

CSV_FILENAME = "multichannel_lidar_scan.csv"

def animate_lidar(filename=CSV_FILENAME):
    df = pd.read_csv(filename)
    df = df[df["Dist_mm"] > 0]

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="ToF Time-Series Animation", width=1280, height=720)

    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    timestamps = sorted(df["Timestamp"].unique())

    for ts in timestamps:
        current_frame = df[df["Timestamp"] == ts]
        points = current_frame[["X", "Y", "Z"]].values

        signals = current_frame["Signal"].values
        colours = np.zeros((len(signals), 3))
        # Green: Max signal intensity
        colours[:, 1] = signals / (signals.max() + 1e-6)

        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility. Vector3dVector(colours)

        pcd = pcd.voxel_down_sample(voxel_size=10.0)

        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        time.sleep(0.05)

    vis.destroy_window()

def visualize_lidar(filename=CSV_FILENAME):
    df = pd.read_csv(filename)
    df = df[df["Dist_mm"] > 0]

    points = df[["X", "Y", "Z"]].values

    # Using Signal strength for color mapping
    signals = df["Signal"].values
    # Normalizing
    norm_signals = (signals - signals.min()) / (signals.max() - signals.min())

    colours = np.zeros((len(norm_signals), 3))
    colours[:, 1] = norm_signals # Green: Stronger
    colours[:, 0] = 1 - norm_signals # Red: Weaker

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colours)
    # Use voxels to clear up noise
    pcd = pcd.voxel_down_sample(voxel_size=10.0)

    print(f"Visualizing {len(points)} points...")
    o3d.visualization.draw_geometries([pcd], window_name="Please God don't just be a blob", width=1280, height=720)

if __name__ == "__main__":
    #animate_lidar()
    visualize_lidar()
