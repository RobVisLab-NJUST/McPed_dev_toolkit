# McPed
We propose a multi-modality ground to air cross-view pose estimation dataset for field robots, and the following figure shows its fusion process and fusion results. There are eight sets of files in the dataset in the example folder,which contain urban and non-urban scenes examples.

![image](https://github.com/RobVisLab-NJUST/McPed_dev_toolkit/blob/main/image%20and%20point%20cloud%20fusion%20process.png)

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

## save_npy.py

RGB map ,density map and elevation map
```javascript
r_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_r
g_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_g
b_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_b
density_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = np.minimum(1.0, np.log(repeat_time + 1)/np.log(128))
elevation_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = max_height
```
![image](https://github.com/RobVisLab-NJUST/McPed_dev_toolkit/blob/main/map.png)

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


## path_visualization.cpp
Reading satellite images using the opencv library.
```javascript
mat1 = cv::imread("image_path");

```
Transformation from a mapping coordinate system to a satellite pixel coordinate system.
```javascript
trans_x = tf->transforms[0].transform.translation.x;
trans_y = tf->transforms[0].transform.translation.y;
trans_x_rot = cos(theta)*(trans_x)-sin(theta)*trans_y;
trans_y_rot = sin(theta)*(trans_x)+cos(theta)*trans_y;
tran_x_pixel = trans_x_rot/0.1961;
tran_y_pixel = trans_y_rot/0.1961;
x_now-=tran_y_pixel;
y_now-=tran_x_pixel;
```

Path Visualization.
```javascript
cv::circle(mat1,cv::Point2f(x_now,y_now),1,cv::Scalar(255,0,0),1);
cv::imshow("PointsinImage", mat1);
cv::waitKey(1);
```

![image](https://github.com/RobVisLab-NJUST/McPed_dev_toolkit/blob/main/path_visualization.png)

We give sample data and labeling in different scenarios for the researcher's testing of the dataset.

non-urban scene:https://drive.google.com/file/d/1l2nlhMmvJwlYWJiipmk4JAaeTYoHE5BU/view?usp=sharing

urban scene:https://drive.google.com/file/d/1YWpqMO_k32dYBMzcXgUfjOrqycsVh2ze/view?usp=drive_link

For the unzip password, please contact:yuanxia@njust.edu.cn
