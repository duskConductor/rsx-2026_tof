import pandas as pd
import numpy as np
import time
import open3d as o3d
import plotly.graph_objects as go
from scipy.spatial import Delaunay

CSV_FILENAME = "raw_pointcloud.csv"

df = pd.read_csv(CSV_FILENAME)

def row_to_points(row):
    xs, ys, zs = [], [], []

    for zone in range(1, 65):
        status = row[f"zone{zone}_status"]

        if status != 1:
            continue

        xs.append(row[f"zone{zone}_x"])
        ys.append(row[f"zone{zone}_y"])
        zs.append(row[f"zone{zone}_z"])

        return np.array(xs), np.array(ys), np.array(zs)

all_x, all_y, all_z = [], [], []

for _, row in df.iterrows():
    xs, ys, zs = row_to_points(row)
    all_x.append(xs)
    all_y.append(ys)
    all_z.append(zs)


all_x = np.concatenate(all_x)
all_y = np.concatenate(all_y)
all_z = np.concatenate(all_z)

sensor_labels = []

for _, row in df.iterrows():
    xs, ys, zs = row_to_points(row)
    sensor_name = row["sensor"]
    sensor_labels.extend([sensor_name] * len(xs))

# Doing colour on point cloud by sensor so I can double-check
# whether the sensors are working correctly.
sensor_labels = np.array(sensor_labels)
sensor_ids = pd.Categorical(sensor_labels).codes

fig = go.Figure(
    data=[
        go.Scatter3d(
            x=all_x,
            y=all_y,
            z=all_z,
            mode="markers", 
            marker=dict(
                size=3,
                color=sensor_ids,
                colorscale="Turbo",
                colorbar=dict(title="Sensor")
            ),
        )
    ]
)

fig.update_layout(
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        aspectmode="data",
    ),
    title="Combined point cloud (all sensors)"
)

fig.show()

# --------------------------------------------------

points_xy = np.vstack([all_x, all_y]).T
tri = Delaunay(points_xy)

i, j, k = tri.simplices.T 

fig = go.Figure(
    data=[
        go.Mesh3d(
            x=all_x,
            y=all_y,
            z=all_z,
            i=i,
            j=j,
            k=k,
            color="lightblue",
            opacity=0.50
        )
    ]
)

fig.update_layout(
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
        aspectmode="data",
    ),
    title="Merged mesh from all sensors"
)

fig.show()

