import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

CSV_PATH = "tof_data.csv"

def create_mesh_from_csv():
    df = pd.read_csv(CSV_PATH)
    distances = df.iloc[-1,1:].values.astype(float)

    fov_deg = 45
    res = 8

    points = []

    for i in range(res):
        for j in range(res):
            idx = i * res + j
            d = distances[idx]

            # NOTE: Distance can't be negative;  330mm is around 12 inches; the plate has a diameter of 12 inches
            # We shouldn't get anything less than 0 or anything more than 12 inches
            if d > 0 and d < 700:
            # if d > 0 and d < 2000:
                angle_x = np.radians((j - res/2 + 0.5) * (fov_deg / res))
                angle_y = np.radians((i - res/2 + 0.5) * (fov_deg / res))

                # Polar to Cartesian
                x = d * np.tan(angle_x)
                y = d * np.tan(angle_y)
                z = d

                points.append([x, y, z])

    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.array(points))

    pcd.estimate_normals()
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=10)
    densities = np.asarray(densities)

    densities_normalized = (densities - densities.min()) / (densities.max() - densities.min())

    colour_map = plt.get_cmap("plasma")
    mesh_colours = colour_map(densities_normalized)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(mesh_colours)

    print("Displaying mesh...")

    o3d.visualization.draw_geometries([mesh])

if __name__ == "__main__":
    create_mesh_from_csv()