import numpy as np
import os
import os.path as osp
import argparse
from pypcd4 import PointCloud
import time
import json
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import sys
import os
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import matplotlib

# Get the parent directory of the current file
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Add it to the sys.path
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from gt_generator.extrinsic_calib import T_lidar_ti, T_box_ti, T_box_lidar, \
                                         opti2sensor_gt, create_transformation_matrix, \
                                         src2dst_transformation

def load_pcl(path, type):
    cloud = None
    if (type == "pcd"):
        pcd = PointCloud.from_path(path)
        cloud = pcd.numpy(("x", "y", "z"))
    else:
        pcd = np.loadtxt(path)
        cloud = pcd
    return cloud


def load_data(frame):
    file_01, sen1_fmt, file_02, sen2_fmt = frame

    # Load sensor pcl 1
    cloud_01 = load_pcl(file_01, sen1_fmt)
    cloud_01[:, [0, 1]] = cloud_01[:, [1, 0]]

    # Load sensor pcl 2
    cloud_02 = load_pcl(file_02, sen2_fmt)

    # point in dst coordinate, reshape to (N, 3)
    cloud_02 = src2dst_transformation(np.linalg.inv(T_lidar_ti), cloud_02)
    cloud_02 = cloud_02[cloud_02[:, 0] <= 7.0]
    cloud_02 = cloud_02[cloud_02[:, 0] >= 0.5]
    cloud_02 = cloud_02[cloud_02[:, 1] <= 1.5]
    cloud_02 = cloud_02[cloud_02[:, 1] >= -1.5]
    cloud_02 = cloud_02[cloud_02[:, 2] <= 2.0]
    cloud_02 = cloud_02[cloud_02[:, 2] >= -1.0]

    return cloud_01, cloud_02

def make_update(ax):
    def update(frame):
        st = time.time()
        points_a, points_b = load_data(frame)

        # Clear previous scatter plots and re-plot
        ax.collections.clear()
        ax.scatter(points_a[:,0], points_a[:,1], points_a[:,2], c='blue', label='Set A', s=1.0)
        ax.scatter(points_b[:,0], points_b[:,1], points_b[:,2], c='red', label='Set B', s=1.0)

        ax.legend()
        plt.pause(0.001)
        return []
    return update

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_pcl2pcl.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
    )

    parser.add_argument(
        "--dir_01",
        type=str,
        help="Path to 1st pointcloud directory",
    )

    parser.add_argument(
        "--dir_02",
        type=str,
        help="Path to 2nd pointcloud directory",
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

    # Get mapping data
    sensor_01 = args.dir_01[args.dir_01.rfind("/")+1:]
    sensor_02 = args.dir_02[args.dir_02.rfind("/")+1:]
    sen2sen_map = "calib/" + sensor_01 + "2" + sensor_02 + "_mapping.txt"
    # sen2gt_map = "calib/" + sensor_01 + "2gt_mapping.txt" 

    # format
    sen1_fmt = "pcd" if os.listdir(osp.join(args.dir,args.dir_01))[0][-3:] == "pcd" else "xyzdi"
    sen2_fmt = "pcd" if os.listdir(osp.join(args.dir,args.dir_02))[0][-3:] == "pcd" else "xyzdi"

    # Load gt if needed
    sen2sen = np.loadtxt(osp.join(args.dir, sen2sen_map), dtype=str)
    # if (args.gt != ""):
    #     gt = osp.join(args.dir, args.gt)
    #     gt_csv = None
    #     with open(gt, "r") as f:
    #         gt_csv = f.read().split("\n")[1:-1] # Load, split by row and remove 1st and last row
    #     # gt_idx = {Timestamp: index}
    #     gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}
    #     sen2gt = np.loadtxt(osp.join(args.dir, sen2gt_map), dtype=str)
    #     ts_map = np.hstack((sen2sen, np.expand_dims(sen2gt[:,1], axis=1)))
    # else:
    ts_map = sen2sen
    # Create ts_map for all available sensor data and gt if needed
    
    fig = plt.figure(figsize=(12,10))
    fig.tight_layout()

    ax11 = fig.add_subplot(111, projection="3d")
    ax11.view_init(elev=10, azim=-165)

    # Initial empty plots
    scat_a = ax11.scatter([], [], [], c='blue', label='Set A')
    scat_b = ax11.scatter([], [], [], c='red', label='Set B')

    ax11.set_xlabel('X-metre')
    ax11.set_ylabel('Y-metre')
    ax11.set_zlabel('Z-metre')

    ax11.set_xlim(0, 7.0)
    ax11.set_ylim(-3, 3)
    ax11.set_zlim(-2.5, 2.5)


    # forming frame list
    frame_list = []
    for i, src2dst in enumerate(ts_map[args.start_frame:]):
        if (src2dst[0] == "nan" or src2dst[1] == "nan"):
            continue

        file_01 = src2dst[0] + "." + sen1_fmt
        file_01 = os.path.join(args.dir, args.dir_01, file_01)
        file_02 = src2dst[1] + "." + sen2_fmt
        file_02 = os.path.join(args.dir, args.dir_02, file_02)

        frame_list.append([file_01, sen1_fmt, file_02, sen2_fmt])

    # Start the animation
    ani = FuncAnimation(fig, make_update(ax11), frames=frame_list, interval=1, blit=False)
    plt.show()

        # Load gt if needed
        # cloud_gt = None
        # if (args.gt != ""):
        #     gt = gt_csv[gt_idx[src2dst[2]]].split(",")[2:-1]
        #     if ("" in gt):
        #         continue
        #     gt = np.array(gt, np.float32)
        #     T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
        #     gt = gt[:-7]

        #     # point in world coordinate, reshape to (N, 3)
        #     points_world = gt.reshape(-1, 3)
        #     cloud_gt = opti2sensor_gt(T_opti_box, T_box_ti, points_world).T
        #     cloud_gt = cloud_gt[:,:3]
        #     cloud = np.append(cloud, cloud_gt, axis=0)
        #     cloud_shape = (cloud_01.shape[0], cloud_02.shape[0], cloud_gt.shape[0])

        # cloud = cloud[cloud[:,2] <= 1.0]

if __name__ == "__main__":
    main()
