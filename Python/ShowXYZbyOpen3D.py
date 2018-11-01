import open3d 
import numpy as np 
pcd = open3d.read_point_cloud("cloud.xyz");
points = np.asarray(pcd.points)
print("points: ", points.shape)
open3d.draw_geometries([pcd])
