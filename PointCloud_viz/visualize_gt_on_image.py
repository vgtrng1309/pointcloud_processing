import numpy as np
import os
import cv2
import argparse
import json
from scipy.spatial.transform import Rotation

left_cam_info = {
    "height": 360,
    "width": 640,
    "distortion_model": "rational_polynomial",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [473.0513610839844,               0.0, 333.80499267578125,
                        0.0, 473.0513610839844, 180.10379028320312,
                        0.0,               0.0,                1.0],
    "r": [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0],
    "p": [473.0513610839844,               0.0, 333.80499267578125, 0.0,
                        0.0, 473.0513610839844, 180.10379028320312, 0.0, 
                        0.0,               0.0,                1.0, 0.0]
}

right_cam_info = {
    "height": 360,
    "width": 640,
    "distortion_model": "rational_polynomial",
    "d": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "k": [473.0513610839844,               0.0, 333.80499267578125,
                        0.0, 473.0513610839844, 180.10379028320312,
                        0.0,               0.0,                1.0],
    "r": [1.0, 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0],
    "p": [473.0513610839844,               0.0, 333.80499267578125, -56.79176330566406,
                        0.0, 473.0513610839844, 180.10379028320312, 0.0, 
                        0.0,               0.0,                1.0, 0.0]
}

def opti2sensor_gt(T_opti_box, T_box_sensor, gt_opti):
    gt_opti = np.array([[gt_opti[0], gt_opti[1], gt_opti[2], 1]]).T
    return np.linalg.inv(T_box_sensor) @ np.linalg.inv(T_opti_box) @ gt_opti

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
        "-g", "--groundtruth",
        type="str",
        help="Path to groundtruth csv file"
    )

    args = parser.parse_args()
    point = np.array([-0.137639, 1.335311, -0.466332])
    T_opti_box = create_transformation_matrix([0.000915, -0.000241, 8e-06, -1.0], \
                                              [-0.172009, 1.887376, 3.303823])
    T_box_cam = create_transformation_matrix([0.500, 0.500, -0.500, 0.500], \
                                             [0.087, -0.717, -0.106])
    
    point = opti2sensor_gt(T_opti_box, T_box_cam, point)
    print(point)
    

if __name__ == "__main__":
    main()

