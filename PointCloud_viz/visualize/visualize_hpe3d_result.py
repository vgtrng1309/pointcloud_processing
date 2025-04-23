# -*- coding: utf-8 -*-
"""
Created on Mon Mar 17 14:59:44 2025

@author: trung.l.nguyen@tuni.fi
"""

"""
- Visualize ground truth and prediction as skeleton 
  with the synced camera view and radar pcl
"""
import numpy as np 
from sklearn import metrics
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from PIL import Image
import os
import os.path as osp
import argparse
import time
import json

image_path_tail = "images"
rad2img_calib_path_tail = "calib/radarpcl2images_mapping.txt"
rad2lid_calib_path_tail = "calib/radarpcl2lidar_mapping.txt"
rad_mars_path_tail = "radarpcl_mars"
lidar_path_tail = "lidar"

def create_argparse():
    parser = argparse.ArgumentParser(
        prog="visualize_hpe3d_result.py",
        description="Visualize 3D human pose estimation result"
    )

    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to model's estimation",
    )

    parser.add_argument(
        "--use_lidar",
        action="store_true",
        help="Decide to show lidar instead of skeleton"
    )

    parser.add_argument(
        "--savefig",
        action="store_true",
        help="Decide to save figure"
    )

    return parser

def main():
    parser = create_argparse()
    args = parser.parse_args()

    # Get radarpcl and gt
    radpcls = np.load("../evaluation/featuremap_test.npy")
    gts = np.load("../evaluation/labels_test.npy")
    preds = np.load(args.file)
    print(radpcls.shape, gts.shape, preds.shape)
    
    # Joints line
    n_joints = 15
    line_idx = np.array([[0,1],[1,2],[2,3],[0,4],[4,5],[5,6],[1,4],
                     [1,9],[4,12],[0,7],[9,12],[8,7],
                     [7,9],[9,10],[10,11],[7,12],[12,13],[13,14]])

    # 
    fig = plt.figure(figsize=(12,10))
    fig.tight_layout()

    ax11 = fig.add_subplot(121, projection="3d")
    ax11.view_init(elev=10, azim=-165)
    ax12 = fig.add_subplot(122, projection="3d")
    ax12.view_init(elev=10, azim=-165)

    show_img = None
    for i, (radpcl, gt, pred) in enumerate(zip(radpcls, gts, preds)):
        # image = plt.imread(osp.join(args.dir, image_path_tail, ts_tuple[1]+".png"))
        # image = np.flip(image, axis=1)
        # if (show_img is None):
        #     show_img = ax11.imshow(image)
        # else:
        #     show_img.set_data(image)

        pcl = radpcl.reshape(-1, 5)
        gt = gt.reshape(-1, 3)
        pred = pred.reshape(-1, 3)

        # Draw radar pcl
        ax11.cla()
        ax11.scatter(pcl[:,0], pcl[:,1], pcl[:,2], c=pcl[:,4], s=5, cmap="jet")
        
        # Draw prediction as line
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

        # Draw radar pcl
        ax12.cla()
        # Draw prediction as line
        for id in line_idx:
            ax12.plot([gt[id[0],0], gt[id[1],0]],
                    [gt[id[0],1], gt[id[1],1]],
                    [gt[id[0],2], gt[id[1],2]], c='blue')
            ax12.plot([pred[id[0],0], pred[id[1],0]],
                    [pred[id[0],1], pred[id[1],1]],
                    [pred[id[0],2], pred[id[1],2]], c='red')

        ax12.set_xlabel('X-metre')
        ax12.set_ylabel('Y-metre')
        ax12.set_zlabel('Z-metre')

        ax12.set_xlim(0, 8)
        ax12.set_ylim(-3, 3)
        ax12.set_zlim(-2.5, 2.5)

        # else:
        #     scat.set_array()
        # if (args.savefig):
        #     fig.canvas.draw()
        #     buf = fig.canvas.tostring_rgb()
        #     width, height = fig.canvas.get_width_height()
        #     img = Image.frombytes("RGB", (width, height), buf)
        #     if (args.no_pred):
        #         img.save(osp.join(args.dir, "radpcl_viz",
        #                  "radpcl_viz_"+str(i).zfill(4)+".png"))
        #     else:
        #         img.save(osp.join(args.dir, "hpe_result", 
        #                              "hpe_result_"+str(i).zfill(4)+".png"))
        # else:
        plt.draw()
        plt.pause(1./30)

if __name__ == "__main__":
    main()