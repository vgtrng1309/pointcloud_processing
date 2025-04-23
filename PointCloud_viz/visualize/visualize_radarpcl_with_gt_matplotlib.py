import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path as osp
import argparse
import time
import json

radpcl_path_tail = "radar/radarpcl"

j_select = ["WaistLFront", "WaistRFront", "WaistLBack", "WaistRBack", "BackTop",
            "HeadTop", "LShoulderTop", "LElbowOut", "LWristOut", "RShoulderTop","RElbowOut", 
            "RWristOut", "LKneeOut", "LAnkleOut", "RKneeOut", "RAnkleOut"]

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
    gt_path_tail = "gt/" + seq_name + "_filtered.csv" 
    gt_csv = None
    with open(osp.join(args.dir, gt_path_tail), "r") as f:
        gt_csv = f.read().split("\n")[1:-1] # Load and remove header and last row

    # 
    fig = plt.figure(figsize=(12,10))
    fig.tight_layout()

    ax11 = fig.add_subplot(111, projection="3d")
    ax11.view_init(elev=10, azim=-165)

    # line_idx = np.array([[5,4],[4,9],[9,10],[10,11],[4,6],[6,7],[7,8],
    #                      [9,1],[6,0],[1,0],[1,14],[14,15],[0,12],[12,13]])

    line_idx = np.array([[0,1],[1,2],[2,3],[0,4],[4,5],[5,6],[1,4],
                     [1,9],[4,12],[0,7],[9,12],[8,7],
                     [7,9],[9,10],[10,11],[7,12],[12,13],[13,14]])

    first_frame = True
    for i, csv_line in enumerate(gt_csv):
        if (i < args.start_frame):
            continue

        gt_line = csv_line.split(",")
        ts_tuple = (gt_line[1], gt_line[3])

        # print("Showing:", ts_tuple[0], "-", ts_tuple[1])
        pcl_file = f"{ts_tuple[0]}.xyzdi"
        cloud = np.loadtxt(osp.join(args.dir, radpcl_path_tail, pcl_file), dtype=np.float32)

        cloud[:,[0, 1]] = cloud[:,[1, 0]]

        cloud = cloud[cloud[:, 0] <= 8.0]
        cloud = cloud[cloud[:, 0] >= 0.0]
        cloud = cloud[cloud[:, 1] <= 3.5]
        cloud = cloud[cloud[:, 1] >= -3.5]
        # cloud = cloud[cloud[:, 2] <= 2.5]
        # cloud = cloud[cloud[:, 2] >= -2.5]

        cloud = cloud[cloud[:,4] >= 15.0] 

        print(np.min(cloud[:,4]), np.max(cloud[:,4]))

        # remove doppler and intensity
        # cloud = cloud[:,:3]

        # GT points
        gt = np.array(gt_line[4:-8]).astype(np.float32).reshape(-1,3)
        print(gt.shape)

        ax11.cla()
        sc = ax11.scatter(cloud[:,0], cloud[:,1], cloud[:,2], c=cloud[:,4], s=5, cmap="jet")
        # ax11.scatter(gt[:,0], gt[:,1], gt[:,2], s=5, c='blue')
        for id in line_idx:
            ax11.plot([gt[id[0],0], gt[id[1],0]],
                      [gt[id[0],1], gt[id[1],1]],
                      [gt[id[0],2], gt[id[1],2]], c='blue')

        ax11.set_xlabel('X-metre')
        ax11.set_ylabel('Y-metre')
        ax11.set_zlabel('Z-metre')

        ax11.set_xlim(0, 10)
        ax11.set_ylim(-3, 3)
        ax11.set_zlim(-2.5, 2.5)

        if (first_frame):
            plt.colorbar(sc, ax=ax11)
            first_frame = False
        plt.draw()
        plt.pause(0.001)
        i += 1

if __name__ == "__main__":
    main()
