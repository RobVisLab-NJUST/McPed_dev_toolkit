import numpy as np
import os
import open3d as o3d
from tqdm import tqdm
from point_cloud_projection import process_point_cloud

def generate_bev(points, colors):
    point_rgb = np.concatenate((points, colors), axis=1)
    img_height, img_width = 1024, 1024
    resolution = 0.04
    
    point_rgb[:, 0] = np.int_(-np.floor(point_rgb[:, 0] / resolution) + img_height / 2)
    point_rgb[:, 1] = np.int_(-np.floor(point_rgb[:, 1] / resolution) + img_width / 2)
    
    r_map, g_map, b_map = np.zeros((img_height, img_width)), np.zeros((img_height, img_width)), np.zeros((img_height, img_width))
    height_map, density_map = np.zeros((img_height, img_width)), np.zeros((img_height, img_width))

    indices = np.lexsort((point_rgb[:, 0], point_rgb[:, 1]))
    point_rgb = point_rgb[indices]
    _, unique_indices, count = np.unique(point_rgb[:, 0:2], axis=0, return_index=True, return_counts=True)

    for i in range(unique_indices.shape[0]):
        old_index = unique_indices[i]
        repeat_time = count[i]
        color_r, color_g, color_b = 0, 0, 0
        max_height = -10
        
        for k in range(old_index, old_index + repeat_time):
            color_r += point_rgb[k, 3]
            color_g += point_rgb[k, 4]
            color_b += point_rgb[k, 5]
            z = point_rgb[k, 2]
            if z > max_height:
                max_height = z
        
        color_r, color_g, color_b = int(color_r / repeat_time), int(color_g / repeat_time), int(color_b / repeat_time)
        r_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_r
        g_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_g
        b_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_b
        density_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = np.minimum(1.0, np.log(repeat_time + 1) / np.log(128))
        height_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = max_height

    RGB_Map = np.zeros((img_height, img_width, 5), dtype=np.uint8)
    min_height = 10
    for i in range(1024):
        for j in range(1024):
            if height_map[i, j] < min_height:
                min_height = height_map[i, j]
    
    min_height = 1 - min_height
    for i in range(1024):
        for j in range(1024):
            if height_map[i, j] != 0:
                height_map[i, j] += min_height
    
    height_map *= 50
    RGB_Map[:, :, 0], RGB_Map[:, :, 1], RGB_Map[:, :, 2] = b_map, g_map, r_map
    RGB_Map[:, :, 3], RGB_Map[:, :, 4] = height_map, density_map * 255
    
    return RGB_Map

def save_results(output_npy_path, pts, colors):
    os.makedirs(output_npy_path, exist_ok=True)
    
    for i in tqdm(range(len(pts))):
        bev_img = generate_bev(pts[i], colors[i])
        output_path = os.path.join(output_npy_path, f"{i:06d}.npy")
        np.save(output_path, bev_img)

        # Optional: Visualization of the point cloud (uncomment if needed)
        # pcd1 = o3d.geometry.PointCloud()
        # pcd1.points = o3d.utility.Vector3dVector(pts[i])
        # pcd1.colors = o3d.utility.Vector3dVector(colors[i] / 255.0)  # Normalize colors to 0-1 range
        # vis = o3d.visualization.Visualizer()
        # vis.create_window()
        # vis.get_render_option().point_size = 3
        # vis.get_render_option().background_color = np.asarray([30 / 255, 30 / 255, 30 / 255])
        # vis.add_geometry(pcd1)
        # vis.run()
        # vis.destroy_window()

if __name__ == '__main__':
    file_name_path = "/path/to/file_name.txt"
    input_photo_path = "/path/to/image"
    input_point_path = "/path/to/lidar"
    output_npy_path = "/path/to/npy"

    # Load camera parameters and intrinsic/extrinsic matrix
    intrinsic = [1429.423, 0, 968.8917, 0, 1428.798, 546.6601, 0, 0, 1]
    extrinsic = [-0.00297489, -0.999994, -0.00192122, 0.0259689,
                 -0.00512753, 0.00193646, -0.999985, 0.198088,
                 0.999982, -0.002965, -0.00513325, 0.779633, 0, 0, 0, 1]

    # Process point cloud and colors from the first file
    pts, colors = process_point_cloud(file_name_path, input_photo_path, input_point_path, intrinsic, extrinsic)

    # Save results as BEV maps and optionally visualize
    save_results(output_npy_path, pts, colors)
