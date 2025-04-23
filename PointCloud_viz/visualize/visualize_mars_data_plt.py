import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path as osp
import argparse
import time
import json

radpcl_path_tail = "radarpcl_mars"

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
        "-t", "--type",
        type=str,
        default="train",
        help="Type"
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

    # seq name
    seq = args.dir[args.dir.rfind("/")+1:]
    ft_path_tail = f"radarpcl_mars/featuremap_{seq}_{args.type}.npy"
    lb_path_tail = f"radarpcl_mars/labels_{seq}_{args.type}.npy"

    fts = np.load(osp.join(args.dir, ft_path_tail))
    fts = fts.reshape(-1, fts.shape[1]**2, 5)
    lbs = np.load(osp.join(args.dir, lb_path_tail)).reshape(-1, 15, 3)
    i = 0

    # 
    fig = plt.figure(figsize=(12,10))
    fig.tight_layout()

    ax11 = fig.add_subplot(111, projection="3d")
    ax11.view_init(elev=10, azim=-165)

    line_idx = np.array([[0,1],[1,2],[2,3],[0,4],[4,5],[5,6],[1,4],
                     [1,9],[4,12],[0,7],[9,12],[8,7],
                     [7,9],[9,10],[10,11],[7,12],[12,13],[13,14]])
    i = 0
    first_frame = True
    for ft, lb in zip(fts, lbs):
        if (i < args.start_frame):
            i += 1
            continue

        cloud = ft
        cloud = cloud[cloud[:,4] >= 30]
        gt = lb
        print(i)

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

        ax11.set_xlim(0, 8)
        ax11.set_ylim(-3, 3)
        ax11.set_zlim(-2, 2)

        if (first_frame):
            plt.colorbar(sc, ax=ax11)
            first_frame = False
        plt.draw()
        plt.pause(0.001)
        i += 1
        # btnpress = plt.waitforbuttonpress(0.1)
        # if btnpress: 
        #     plt.waitforbuttonpress(-1)

if __name__ == "__main__":
    main()
