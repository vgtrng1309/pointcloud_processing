import numpy as np
import os
import cv2
import argparse
import json
from scipy.spatial.transform import Rotation
from pypcd4 import PointCloud

left_cam_info = {
    "height": 576,
    "width": 1024,
    "distortion_model": "rational_polynomial",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [756.9974365234375,               0.0, 534.0882568359375,
                        0.0, 756.9974365234375, 288.1669311523438,
                        0.0,               0.0,               1.0],
    "r": [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0],
    "p": [756.9974365234375,               0.0, 534.0882568359375, 0.0,
                        0.0, 756.9974365234375, 288.1669311523438, 0.0, 
                        0.0,               0.0,               1.0, 0.0]
}

old_left_cam_info = {
    "height": 360,
    "width": 640,
    "distortion_model": "rational_polynomial",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [473.0513610839844,               0.0, 333.80499267578125,
                        0.0, 473.0513610839844, 180.10379028320312,
                        0.0,               0.0,               1.0],
    "r": [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0],
    "p": [473.0513610839844,               0.0, 333.80499267578125, 0.0,
                        0.0, 473.0513610839844, 180.10379028320312, 0.0, 
                        0.0,               0.0,               1.0, 0.0]
}

right_cam_info = {
    "height": 576,
    "width": 1024,
    "distortion_model": "rational_polynomial",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [756.9974365234375,               0.0, 534.0882568359375,
                        0.0, 756.9974365234375, 288.1669311523438,
                        0.0,               0.0,               1.0],
    "r": [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0],
    "p": [756.9974365234375,               0.0, 534.0882568359375, -90.88066101074219,
                        0.0, 756.9974365234375, 288.1669311523438, 0.0, 
                        0.0,               0.0,               1.0, 0.0]
}

def point2pixel(point, cam_info):
    # Define the 3D points (N, 3)
    points_3d = np.array(point, np.float32) 

    # Transform point from custom coordinator to original
    T_org_custom = np.array([[0., -1., 0., 0.],
                             [0., 0., -1., 0.],
                             [1., 0., 0., 0.],
                             [0., 0., 0., 1.]])
    # T_org_custom = np.eye(4, 4)
    
    # (4, N)
    points_3d = T_org_custom @ np.hstack((points_3d, np.ones((points_3d.shape[0],1)))).T
    
    # (N, 3)
    points_3d = points_3d.T[:,:3]
    
    # Define the rotation and translation vectors, Extrinsic to world coordination
    rvec = np.zeros((3, 1), np.float32) 
    tvec = np.zeros((3, 1), np.float32)
    
    # Map the 3D point to 2D point
    points_2d, _ = cv2.projectPoints(points_3d, 
                                    rvec, tvec, 
                                    np.array(cam_info["k"]).reshape(3, 3), 
                                    None)

    return points_2d

def opti2sensor_gt(T_opti_box, T_box_sensor, points):
    points = np.hstack((points, np.ones((points.shape[0], 1)))).T
    
    # (4,4) @ (4,4) @ (4, N) => (4, N)
    return np.linalg.inv(T_box_sensor) @ np.linalg.inv(T_opti_box) @ points

def create_transformation_matrix(R, t):
    T_mat = np.eye(4, 4)
    R = np.asarray(R)
    t = np.asarray(t)

    R = Rotation.from_quat(R).as_matrix()
    T_mat[:3,:3] = R
    T_mat[:3,3] = t
    return T_mat

def load_extrinsic(path):
    data = None
    with open(path, "r") as f:
        data = json.load(f)
    return data

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_gt_on_image.py",
        description="For visualizing gt on image"
    )

    parser.add_argument(
        "-d", "--data_dir",
        type=str,
        help="Path to image directory"
    )

    parser.add_argument(
        "-p", "--pcl_path",
        type=str,
        help="Path to point cloud file. Can be pcd, xyzdi or groundtruth"
    )

    parser.add_argument(
        "--mapping_file",
        type=str,
        help="Path to timestamp mapping file"
    )

    parser.add_argument(
        "-s", "--start_frame",
        type=int,
        default=0,
        help="Starting frame"
    )

    parser.add_argument(
        "-m", "--frame_by_frame_mode",
        action="store_true",
        help="Load frame by frame"
    )


    args = parser.parse_args()
    
    # Get image list
    img_list = sorted(os.listdir(args.data_dir))

    # Get pcl list
    if (".csv" in args.pcl_path):
        gt_csv = None
        with open(args.pcl_path, "r") as f:
            gt_csv = f.read().split("\n")[1:-1] # Load, split by row and remove 1st and last row
        gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}

        # T_box_cam = create_transformation_matrix([0.5, 0.5, -0.5, 0.5], \
        #                                          [0.151, -0.781, -0.186])
        T_box_cam = create_transformation_matrix([0.5, 0.5, -0.5, 0.5], \
                                                 [0.25, -0.761, -0.155])

    else:
        T_lidar_cam = create_transformation_matrix([0.970, 0.242, -0.002, 0.005], \
                                                #    [0.149, -0.086, -0.745])
                                                   [0.145, -0.078, -0.745])
        T_opti_box = create_transformation_matrix([0., 0., 0., 1.], \
                                                  [0., 0., 0.])

    # Get mapping data
    ts_map = np.loadtxt(args.mapping_file, dtype=str)


    for i, img2gt in enumerate(ts_map):
        if (i < args.start_frame):
            continue
        if (img2gt[1] == "nan"):
            continue
        # print("Timestamp: ", img2gt[0], "-", gt_csv[gt_idx[img2gt[1]]].split(",")[1])
        print("Timestamp: ", img2gt[0], img2gt[1], abs(float(img2gt[0]) - float(img2gt[1])))
        # try:
        img = cv2.imread(os.path.join(args.data_dir, img2gt[0]+".png"))

        if (".csv" in args.pcl_path):
            gt = np.array(gt_csv[gt_idx[img2gt[1]]].split(",")[2:-1], np.float32)
            T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
            gt = gt[:-7]

            #point in world coordinate, reshape to (N, 3)
            points_world = gt.reshape(-1, 3)
            points_cam = opti2sensor_gt(T_opti_box, T_box_cam, points_world).T
        else:
            pcd = PointCloud.from_path(os.path.join(args.data_dir.replace("images", "lidar"), img2gt[1]+".pcd"))
            points_world = pcd.numpy(("x", "y", "z"))
            points_world = points_world[~np.isnan(points_world).any(axis=1)]
            points_cam = opti2sensor_gt(T_opti_box, T_lidar_cam, points_world).T

        print(points_cam)

        points_cam = points_cam[points_cam[:, 0] <= 7.0]
        points_cam = points_cam[points_cam[:, 0] >= 1.0]

        points_cam = points_cam[points_cam[:, 2] <= 0.5]
        points_cam = points_cam[points_cam[:, 2] >= -1.0]

        # points_cam = points_cam[points_cam[:, 1] <= 0.5]
        # points_cam = points_cam[points_cam[:, 1] >= -0.5]


        points_img = point2pixel(points_cam[:,:3], old_left_cam_info)

        img = cv2.flip(img, 0)
        for point_2d in points_img:
            point_2d = point_2d[0]

            # TODO: Hard code here
            # point_2d[0] -= int(90.88066101074219/2)
            cv2.circle(img, point_2d.astype(np.int), 3, (0, 0, 255), -1)

        img = cv2.flip(img, 0)
        cv2.imshow("Test", img)
        if (args.frame_by_frame_mode):
            key = cv2.waitKey(0) 
        else:
            key = cv2.waitKey()
        if (key == ord("q")):
            break
        # except:
        #     print("Point cloud missing")
    

if __name__ == "__main__":
    main()

