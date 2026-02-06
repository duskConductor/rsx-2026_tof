import pandas as pd
df = pd.read_csv("raw_pointcloud.csv")
print(df["sensor"].value_counts())