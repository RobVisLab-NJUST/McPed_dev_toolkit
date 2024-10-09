import numpy as np
import cv2 as cv
import open3d as o3d
import math

def get_uv(matrix_in, matrix_out, x, y, z):
    matrix3 = np.array([[x], [y], [z], [1]])
    result = np.dot(np.dot(matrix_in, matrix_out), matrix3)
    u = result[0, 0]
    v = result[1, 0]
    depth = result[2, 0]
    u = u / depth
    v = v / depth
    return u, v

def get_color(matrix_in, matrix_out, x, y, z, row, col, color_vec):
    u, v = get_uv(matrix_in, matrix_out, x, y, z)
    index = int(v) * col + int(u)
    r = color_vec[index, 0]
    g = color_vec[index, 1]
    b = color_vec[index, 2]
    return r, g, b

def process_point_cloud(file_name_path, input_photo_path, input_point_path, intrinsic, extrinsic):
    matrix1 = np.array([[intrinsic[0], intrinsic[1], intrinsic[2]],
                        [intrinsic[3], intrinsic[4], intrinsic[5]],
                        [intrinsic[6], intrinsic[7], intrinsic[8]]])
    
    matrix2 = np.array([[extrinsic[0], extrinsic[1], extrinsic[2], extrinsic[3]],
                        [extrinsic[4], extrinsic[5], extrinsic[6], extrinsic[7]],
                        [extrinsic[8], extrinsic[9], extrinsic[10], extrinsic[11]]])

    camera_matrix = np.eye(3, dtype=float)
    camera_matrix[0, 0] = intrinsic[0]
    camera_matrix[0, 2] = intrinsic[2]
    camera_matrix[1, 1] = intrinsic[4]
    camera_matrix[1, 2] = intrinsic[5]

    distortion_coef = np.zeros((5, 1), dtype=float)
    distortion_coef[0, 0] = intrinsic[9]  # Change index based on correct distortion parameter
    distortion_coef[1, 0] = intrinsic[10]
    distortion_coef[2, 0] = intrinsic[11]
    distortion_coef[3, 0] = intrinsic[12]
    distortion_coef[4, 0] = intrinsic[13]

    files_vec = []
    with open(file_name_path, "r") as fp:
        for line in fp:
            files_vec.append(line.strip())

    points_list = []
    colors_list = []

    for i in range(len(files_vec)):
        # Load photo and point cloud
        timestamp = files_vec[i][0:20]
        timestamp_img = files_vec[i][0:20]
        photo_path = input_photo_path + '/' + timestamp_img + '.jpg'
        cloud_path = input_point_path + '/' + timestamp + '.pcd'

        src_img = cv.imread(photo_path)
        width, height = src_img.shape[1], src_img.shape[0]

        # Undistort the image using the camera matrix and distortion coefficients
        new_cam_matrix, valid_roi = cv.getOptimalNewCameraMatrix(camera_matrix, distortion_coef, (width, height), 1, (width, height))
        map1, map2 = cv.initUndistortRectifyMap(camera_matrix, distortion_coef, None, new_cam_matrix, (width, height), cv.CV_32FC1)
        src_img = cv.remap(src_img, map1, map2, cv.INTER_LINEAR)

        # Convert image to color vector
        color_vec = np.zeros((width * height, 3), dtype=np.uint8)
        for i in range(height):
            for j in range(width):
                color_vec[i * width + j, 0] = src_img[i, j, 2]  # R
                color_vec[i * width + j, 1] = src_img[i, j, 1]  # G
                color_vec[i * width + j, 2] = src_img[i, j, 0]  # B

        # Load point cloud and filter points
        pcd = o3d.io.read_point_cloud(cloud_path)
        pts = np.asarray(pcd.points)

        # Select points based on some criteria (same logic as original)
        cond = np.ones(pts.shape[0], dtype=bool)
        for d in range(pts.shape[0]):
            x, y, z = pts[d, 0], pts[d, 1], pts[d, 2]
            if math.fabs(y / x) > 0.73:
                cond[d] = False
            if math.sqrt(x**2 + y**2) > 20:
                cond[d] = False
            if z > 1.4:
                cond[d] = False

        # Get corresponding color for each point
        colors = np.zeros([pts.shape[0], 3])
        for k in range(pts.shape[0]):
            x, y, z = pts[k, 0], pts[k, 1], pts[k, 2]
            r, g, b = get_color(matrix1, matrix2, x, y, z, height, width, color_vec)
            colors[k] = [r, g, b]

        points_list.append(pts[cond, :])
        colors_list.append(colors[cond, :])

    return points_list, colors_list
