# McPed
We propose a multi-modality ground to air cross-view pose estimation dataset for autonomous driving, and the following figure shows its fusion process and fusion results. Tere are eight sets of files in the dataset in the example folder,which contain urban and non-urban scenes examples.



## color_point.py
This script provides functions for reading calibration parameters,
Through the internal parameters of the camera and the external parameters between the camera and the LiDAR,
calculate the relationship between the pixels of the image and the points in the point cloud.


The intrinsic parameter and extrinsi parameter matrices follow the following format:
```javascript
matrix_in = np.array([[intrinsic[0], intrinsic[1], intrinsic[2]],
            [intrinsic[3], intrinsic[4], intrinsic[5]],
            [intrinsic[6], intrinsic[7], intrinsic[8]]])
matrix_ex = np.array([[extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]],
            [extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]],
            [extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]]])
```

Where "matrix_in" denotes the internal parameters and "matrix_out" denotes the external parameters,
"matrix3" denotes the augmented matrix representation of the point cloud,"u" and "v" denotes the horizontal and vertical 
coordinates of the points in the point cloud projected into the image, respectively.
```javascript
result = np.dot(np.dot(matrix_in, matrix_ex), matrix3)
u = result[0, 0]
v = result[1, 0]
depth = result[2, 0]
u = u/depth
v = v/depth
```
Get the corresponding RGB value.
```javascript
u, v = get_uv(matrix_in, matrix_ex, x, y, z)
index = int(v) * col + int(u)
r = color_vec[index, 0]
g = color_vec[index, 1]
b = color_vec[index, 2]
```
Color point cloud visualization.
```javascript
pcd = o3d.geometry.PointCloud()
x = np.concatenate((pts, colors), axis=1)
pcd.points = o3d.utility.Vector3dVector(pts)
pcd.colors = o3d.utility.Vector3dVector(colors)
vis = o3d.visualization.Visualizer()
vis.create_window()
# Set the size of the points in the point cloud
vis.get_render_option().point_size = 3
vis.get_render_option().background_color = np.asarray([30/255, 30/255, 30/255])
vis.add_geometry(pcd1)
vis.run()
vis.destroy_window()
```
