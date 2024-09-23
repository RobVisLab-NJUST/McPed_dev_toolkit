import open3d as o3d
import numpy as np
import cv2 as cv
import math
from tqdm import *


def get_uv(matrix_in, matrix_out, x, y, z):
    matrix3 = np.array([[x], [y], [z], [1]])
    result = np.dot(np.dot(matrix_in, matrix_out), matrix3)
    u = result[0, 0]
    v = result[1, 0]
    depth = result[2, 0]
    u = u/depth
    v = v/depth
    return u, v


def get_color(matrix_in, matrix_out, x, y, z, row, col, color_vec):
    u, v = get_uv(matrix_in, matrix_out, x, y, z)
    index = int(v) * col + int(u)
    r = color_vec[index, 0]
    g = color_vec[index, 1]
    b = color_vec[index, 2]
    return r, g, b


def generate_bev(points, color):
    point_rgb = np.concatenate((points, color), axis=1)
    img_height = 1024
    img_width = 1024
    resolution = 0.04
    point_rgb[:, 0] = np.int_(-np.floor(point_rgb[:, 0]/resolution)+img_height/2)
    point_rgb[:, 1] = np.int_(-np.floor(point_rgb[:, 1]/resolution)+img_width/2)
    r_map = np.zeros((img_height, img_width))
    g_map = np.zeros((img_height, img_width))
    b_map = np.zeros((img_height, img_width))
    height_map = np.zeros((img_height, img_width))
    density_map = np.zeros((img_height, img_width))
    indices = np.lexsort((point_rgb[:, 0], point_rgb[:, 1]))
    point_rgb = point_rgb[indices]
    _, indices, count = np.unique(point_rgb[:, 0:2], axis=0, return_index=True, return_counts=True)
    for i in range(indices.shape[0]):
        old_index = indices[i]
        repeat_time = count[i]
        color_r = 0
        color_g = 0
        color_b = 0
        max_height = -10
        for k in range(old_index, old_index+repeat_time):
            color_r += point_rgb[k, 3]
            color_g += point_rgb[k, 4]
            color_b += point_rgb[k, 5]
            z = point_rgb[k, 2]
            if z > max_height:
                max_height = z
        color_r = int(color_r / repeat_time)
        color_g = int(color_g / repeat_time)
        color_b = int(color_b / repeat_time)
        r_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_r
        g_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_g
        b_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = color_b
        density_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = np.minimum(1.0, np.log(repeat_time + 1)/np.log(128))
        height_map[int(point_rgb[old_index, 0]), int(point_rgb[old_index, 1])] = max_height

    RGB_Map = np.zeros((img_height, img_width, 5), dtype=np.uint8)
    min_height = 10
    for i in range(1024):
        for j in range(1024):
            if height_map[i, j]<min_height:
                min_height = height_map[i, j]
    min_height = 1-min_height
    for i in range(1024):
        for j in range(1024):
            if height_map[i, j]!=0:
                height_map[i, j]+=min_height
    height_map *= 50
    RGB_Map[:, :, 0] = b_map  # r_map
    RGB_Map[:, :, 1] = g_map  # g_map
    RGB_Map[:, :, 2] = r_map  # b_map
    RGB_Map[:, :, 3] = height_map  # height_map
    RGB_Map[:, :, 4] = density_map*255  # density_map
    return RGB_Map


if __name__ == '__main__':
    file_name_path = "/home/kyyyyy/crossschooldata/2023-08-09-13-30-11/file_name.txt"
    input_photo_path = "/home/kyyyyy/crossschooldata/2023-08-09-13-30-11/image"
    input_point_path = "/home/kyyyyy/crossschooldata/2023-08-09-13-30-11/lidar"
    output_bev_path = "/home/kyyyyy/crossschooldata/2023-08-09-13-30-11/color_bev"
    output_npy_path = "/home/kyyyyy/crossschooldata/2023-08-09-13-30-11/npy"

    # load camera parameters and intrinsic, extrinsic parameters
    intrinsic = [1429.423, 0, 968.8917,
                 0, 1428.798, 546.6601,
                 0, 0, 1]
    distortion = [-0.3746, 0.1532, 0, 0, 0]
    # extrinsic = [0.0134988, -0.999897, 0.00492269, -0.0334677,
    #              0.025299,  -0.00458003,  -0.999669,  0.108653,
    #              0.999589, 0.0136188, 0.0252345, 0.611799,
    #              0, 0, 0, 1]
    # if build off-road dataset, use this extrinsic parameters
    # extrinsic = [-0.0157143, -0.999852, 0.00694549, 0.0702408,
    #              0.0220965, -0.00729192, -0.999729, 0.109341,
    #              0.999632, -0.0155566, 0.0222078, 0.682793,
    #              0, 0, 0, 1]
    extrinsic = [-0.00297489, - 0.999994, - 0.00192122, 0.0259689,
                 -0.00512753, 0.00193646, - 0.999985, 0.198088,
                 0.999982, - 0.002965, - 0.00513325, 0.779633,
                 0, 0, 0, 1]

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

    distortion_coef = np.zeros((5,1),dtype=float)
    distortion_coef[0, 0] = distortion[0]
    distortion_coef[1, 0] = distortion[1]
    distortion_coef[2, 0] = distortion[2]
    distortion_coef[3, 0] = distortion[3]
    distortion_coef[4, 0] = distortion[4]
    # print(distortion_coef)

    files_vec = []
    with open(file_name_path, "r") as fp:
        for line in fp:
            files_vec.append(line.strip())
    for i in tqdm(range(len(files_vec))):
        # first load photo and point cloud
        timestamp = files_vec[i][0:20]
        timestamp_img = files_vec[i][0:20]
        photo_path = input_photo_path+'/'+timestamp_img+'.jpg'
        cloud_path = input_point_path+'/'+timestamp+'.pcd'
        src_img = cv.imread(photo_path)
        width = src_img.shape[1]
        height = src_img.shape[0]
        # use intrinsic matrix and distortion matrix to correct the photo
        new_cam_matrix, valid_roi = cv.getOptimalNewCameraMatrix(camera_matrix,distortion_coef,(width,height),1,(width,height))
        map1, map2 = cv.initUndistortRectifyMap(camera_matrix, distortion_coef, None, new_cam_matrix, (width,height),cv.CV_32FC1)
        src_img = cv.remap(src_img, map1, map2, cv.INTER_LINEAR)
        # cv.imshow("test",src_img)
        # cv.waitKey(10000)
        color_vec = np.zeros((width*height,3), dtype=np.uint8)
        for i in range(height):
            for j in range(width):
                color_vec[i * width + j, 0] = src_img[i, j, 2]
                color_vec[i * width + j, 1] = src_img[i, j, 1]
                color_vec[i * width + j, 2] = src_img[i, j, 0]
        pcd = o3d.io.read_point_cloud(cloud_path)
        pts = np.asarray(pcd.points)
        # select points range
        cond = np.ones(pts.shape[0],dtype=bool)
        # print(cond.shape)
        for d in range(pts.shape[0]):
            x = pts[d, 0]
            y = pts[d, 1]
            z = pts[d, 2]
            if math.fabs(y/x) > 0.73:
                cond[d] = False
            distance = math.sqrt(x**2+y**2)
            if distance > 20:
                cond[d] = False
            if z > 1.4:
                cond[d] = False

        colors = np.zeros([pts.shape[0],3])
        for k in range(pts.shape[0]):
            x = pts[k, 0]
            y = pts[k, 1]
            z = pts[k, 2]
            r, g, b = get_color(matrix1, matrix2, x, y, z, height, width, color_vec)
            # colors[k] = [r/255, g/255, b/255]
            colors[k] = [r, g, b]
        colors = colors[cond, :]

        pts = pts[cond, :]
        output_path = output_npy_path + '/' + timestamp + '.npy'
        bev_img = generate_bev(pts, colors)
        np.save(output_path, bev_img)
        # print(bev_img[450:500,500])
        # cv.imshow('test',bev_img[:,:,0:3])
        # cv.waitKey(10)

        # 如果要查看点云，取消以下注释
        # pcd1 = o3d.geometry.PointCloud()
        # x = np.concatenate((pts, colors), axis=1)
        # # cv.imshow('test', bev_img)
        # # cv.imwrite("/home/xjc/test.jpg",bev_img)
        # # output_path = output_bev_path+'/'+timestamp+'.jpg'
        # # cv.imwrite(output_path, bev_img)
        # pcd1.points = o3d.utility.Vector3dVector(pts)
        # pcd1.colors = o3d.utility.Vector3dVector(colors)
        # vis = o3d.visualization.Visualizer()
        # vis.create_window()
        # # 设置点云大小
        # vis.get_render_option().point_size = 3
        # vis.get_render_option().background_color = np.asarray([30/255, 30/255, 30/255])
        # vis.add_geometry(pcd1)
        # # o3d.io.write_point_cloud('/home/xjc/test.pcd', pcd, True)
        # vis.run()
        # vis.destroy_window()
