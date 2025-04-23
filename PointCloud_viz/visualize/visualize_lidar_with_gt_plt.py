import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path as osp
import argparse
import time
import json
from pypcd4 import PointCloud
import sys
import os

# Get the parent directory of the current file
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Add it to the sys.path
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from gt_generator.extrinsic_calib import T_lidar_ti, T_box_ti, T_box_lidar, opti2sensor_gt, create_transformation_matrix

lidar_path_tail = "lidar"
lidar_gt_tail = "calib/lidar2gt_mapping.txt"

j_select = ["WaistLFront", "WaistRFront", "WaistLBack", "WaistRBack", "BackTop",
            "HeadTop", "LShoulderTop", "LElbowOut", "LWristOut", "RShoulderTop","RElbowOut", 
            "RWristOut", "LKneeOut", "LAnkleOut", "RKneeOut", "RAnkleOut"]

def transform_points(T, points):
    # reshape points from (-1, 3) to (3, -1)
    points = points.T

    # create homogenous points (4, -1)
    points_h = np.ones((4, points.shape[1]))
    points_h[:3, :] = points

    # transform points -> (4, -1)
    points_tf = T @ points_h

    # return (-1, 3)
    return points_tf[:3, :].T

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_pointcloud.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to data dir",
    )

    parser.add_argument(
        "-s", "--start_frame",
        type=int,
        default=0,
        help="Starting frame"
    )

    parser.add_argument(
        "-m", "--mode",
        type=str,
        default="rt",
        help="Mode: rt-realtime, fbf-frame by frame, s-single, v-verification"
    )

    args = parser.parse_args()

    # Get seq name
    seq_name = args.dir[args.dir.rfind("/")+1:]

    # Get gt path tail
    gt_path_tail = "gt/" + seq_name + ".csv" 
    gt_csv = None
    with open(osp.join(args.dir, gt_path_tail), "r") as f:
        gt_csv = f.read().split("\n")[1:-1] # Load and remove header and last row
    gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}

    # Get mapping file
    mapping_file = osp.join(args.dir, lidar_gt_tail)
    ts_map = np.loadtxt(mapping_file, dtype=str)

    # 
    fig = plt.figure(figsize=(12,10))
    fig.tight_layout()

    ax11 = fig.add_subplot(111, projection="3d")
    ax11.view_init(elev=10, azim=-165)
    ax11.set_xlabel('X-metre')
    ax11.set_ylabel('Y-metre')
    ax11.set_zlabel('Z-metre')

    ax11.set_xlim(0, 10)
    ax11.set_ylim(-3, 3)
    ax11.set_zlim(-2.5, 2.5)

    # line_idx = np.array([[5,4],[4,9],[9,10],[10,11],[4,6],[6,7],[7,8],
    #                      [9,1],[6,0],[1,0],[1,14],[14,15],[0,12],[12,13]])

    first_frame = True
    sc01, sc02 = None, None
    for i, ts_tuple in enumerate(ts_map):
        if (i < args.start_frame or "nan" in ts_tuple):
            continue
        print("Showing:", ts_tuple[0], "-", ts_tuple[1])

        # Load pcd file
        pcl_file = f"{ts_tuple[0]}.pcd"
        pcd = PointCloud.from_path(osp.join(args.dir, lidar_path_tail, pcl_file))
        cloud = pcd.numpy(("x", "y", "z"))

        # Transform points in lidar coord to ti coord
        cloud = transform_points(np.linalg.inv(T_lidar_ti), cloud)
        cloud = cloud[cloud[:, 2] > -1.0]
        cloud = cloud[cloud[:, 2] < 1.0]
        cloud = cloud[cloud[:, 0] >= 0.0]
        cloud = cloud[cloud[:, 0] <= 8.0]
        cloud = cloud[cloud[:, 1] >= -2.0]
        cloud = cloud[cloud[:, 1] <= 2.0]


        # cloud = cloud[cloud[:,4] >= 20.0] 

        # # GT points
        gt_line = np.array(gt_csv[gt_idx[ts_tuple[1]]].split(",")[2:-1])
        # Filter out empty value
        gt_line[gt_line == ""] = "nan"
        gt_line = np.array(gt_line).astype(np.float32)
        gt_w = gt_line[:-7].reshape(-1, 3)
        box_rot, box_pos = gt_line[-7:-3], gt_line[-3:]
        
        # Transform GT
        T_opti_box = create_transformation_matrix(box_rot, box_pos)
        gt = opti2sensor_gt(T_opti_box, T_box_ti["april2"], gt_w)

        if (first_frame):
            sc01 = ax11.scatter(cloud[:,0], cloud[:,1], cloud[:,2], s=1, c="red")
            sc02 = ax11.scatter(gt[:,0], gt[:,1], gt[:,2], s=1, c='blue')
            first_frame = False
        else:
            sc01._offsets3d = (cloud[:,0], cloud[:,1], cloud[:,2])
            sc02._offsets3d = (gt[:,0], gt[:,1], gt[:,2])
        # for id in line_idx:
        #     ax11.plot([gt[id[0],0], gt[id[1],0]],
        #               [gt[id[0],1], gt[id[1],1]],
        #               [gt[id[0],2], gt[id[1],2]], c='blue')

        plt.draw()
        plt.pause(0.001)
        # btnpress = plt.waitforbuttonpress(0.1)
        # if btnpress is not None:
        #     plt.waitforbuttonpress(-1)

if __name__ == "__main__":
    main()
