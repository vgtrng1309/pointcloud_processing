import numpy as np
import open3d as o3d
import os
import os.path as osp
import argparse
from pypcd4 import PointCloud
import time
import json
from scipy.spatial.transform import Rotation
import sys 

# Get the parent directory of the current file
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Add it to the sys.path
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
from gt_generator.extrinsic_calib import T_lidar_ti, T_box_ti, T_box_lidar, \
                                 opti2sensor_gt, create_transformation_matrix, \
                                 src2dst_transformation

class Viewer3D(object):
    def __init__(self, mode):
        self.first_cloud = True
        # self.vis = o3d.visualization.VisualizerWithKeyCallback()
        if (mode != "v"):
            if (mode != "s"):
                self.vis = o3d.visualization.Visualizer()
            else:
                self.vis = o3d.visualization.VisualizerWithEditing()

            self.vis.create_window(visible=True)
        
            # Set option. Call only after creating visualizer window.
            opt = self.vis.get_render_option()
            if (mode == "s"):
                opt.show_coordinate_frame = True
            opt.background_color = np.asarray([0.5, 0.5, 0.5])

            # pcl object
            self.pcl = o3d.geometry.PointCloud()
            
            # Draw axis
            # if (mode != "s"):
            #     mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
            #     self.vis.add_geometry(mesh)

        self.last_cloud = None
        # self.labels = []
        self.is_alive = True
        self.mode = mode

    # def update_label(self, cloud):
    #     if (len(self.labels) != 0):
    #         for i in range(len(self.labels)):
    #             self.labels[i].position = cloud[i]   
    #             self.vis.update_geometry(self.labels[i])
    #     else:
    #         for i in range(cloud.shape[0]):
    #             text = Label3D("p"+str(i).zfill(2), cloud[i])
    #             self.labels.append(Label3D)
    #             self.vis.add_geometry(self.labels[i])
        

    def update_cloud(self, cloud, shape):
        # Convert numpy array of 3D points to Vector3d of open3d
        self.pcl.points = o3d.utility.Vector3dVector(cloud)

        pcl01_size, pcl02_size, gt_size = shape
        # if (gt_size != 0):
        np_color = np.zeros((cloud.shape[0],3))
        np_color[:-gt_size,0] = 1.0
        np_color[-gt_size:,2] = 1.0
        np_color[pcl01_size:pcl01_size+pcl02_size,1] = 1.0
        
        self.pcl.colors = o3d.utility.Vector3dVector(np_color)

        if (self.first_cloud):
            self.vis.add_geometry(self.pcl)
            self.first_cloud = False

        self.vis.update_geometry(self.pcl)
        self.vis.poll_events()
        self.vis.update_renderer()

        if (not self.vis.poll_events()):
            self.vis.destroy_window()
            self.is_alive = False

def load_extrinsic(path):
    data = None
    with open(path, "r") as f:
        data = json.load(f)
    return data

def load_pcl(path, type):
    cloud = None
    if (type == "pcd"):
        pcd = PointCloud.from_path(path)
        cloud = pcd.numpy(("x", "y", "z"))
    else:
        pcd = np.loadtxt(path)
        cloud = pcd
    return cloud

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_pointcloud.py",
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
        "-g", "--gt",
        type=str,
        default="",
        help="Path to ground truth if needed",
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
    sen2gt_map = "calib/" + sensor_01 + "2gt_mapping.txt" 

    # format
    src_fmt = "pcd" if os.listdir(osp.join(args.dir,args.dir_01))[0][-3:] == "pcd" else "xyzdi"
    dst_fmt = "pcd" if os.listdir(osp.join(args.dir,args.dir_02))[0][-3:] == "pcd" else "xyzdi"

    # Load gt if needed
    sen2sen = np.loadtxt(osp.join(args.dir, sen2sen_map), dtype=str)
    if (args.gt != ""):
        gt = osp.join(args.dir, args.gt)
        gt_csv = None
        with open(gt, "r") as f:
            gt_csv = f.read().split("\n")[1:-1] # Load, split by row and remove 1st and last row
        # gt_idx = {Timestamp: index}
        gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}
        sen2gt = np.loadtxt(osp.join(args.dir, sen2gt_map), dtype=str)
        ts_map = np.hstack((sen2sen, np.expand_dims(sen2gt[:,1], axis=1)))
    else:
        ts_map = sen2sen
    # Create ts_map for all available sensor data and gt if needed
    # visualizer
    viewer = Viewer3D(args.mode)
    for i, src2dst in enumerate(ts_map):
        if (src2dst[0] == "nan" or src2dst[1] == "nan"):
            continue

        if (ts_map.shape[1] == 3 and src2dst[2] == "nan"):
            continue

        # Load sensor pcl 1
        file_01 = src2dst[0] + "." + src_fmt
        cloud_01 = load_pcl(os.path.join(args.dir, args.dir_01, file_01), src_fmt)
        if (sensor_01 == "radarpcl"):
            cloud_01[:, [0, 1]] = cloud_01[:, [1, 0]]
        if (src_fmt == "xyzdi"):
             # remove doppler and snr
            cloud_01 = cloud_01[:,:3]

        # Load sensor pcl 2
        file_02 = src2dst[1] + "." + dst_fmt
        cloud_02 = load_pcl(os.path.join(args.dir, args.dir_02, file_02), dst_fmt)

        # point in dst coordinate, reshape to (N, 3)
        cloud_02 = src2dst_transformation(np.linalg.inv(T_lidar_ti), cloud_02)
        cloud_02 = cloud_02[cloud_02[:, 0] <= 7.0]
        cloud_02 = cloud_02[cloud_02[:, 0] >= 0.5]
        cloud_02 = cloud_02[cloud_02[:, 1] <= 2.0]
        cloud_02 = cloud_02[cloud_02[:, 1] >= -2.0]
        cloud_02 = cloud_02[cloud_02[:, 2] <= 2.0]

        cloud = np.append(cloud_01, cloud_02, axis=0)
        cloud_shape = (cloud_01.shape[0], cloud_02.shape[0], 0)
        
        # Load gt if needed
        cloud_gt = None
        if (args.gt != ""):
            gt = gt_csv[gt_idx[src2dst[2]]].split(",")[2:-1]
            if ("" in gt):
                continue
            gt = np.array(gt, np.float32)
            T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
            gt = gt[:-7]

            # point in world coordinate, reshape to (N, 3)
            points_world = gt.reshape(-1, 3)
            cloud_gt = opti2sensor_gt(T_opti_box, T_box_ti, points_world).T
            cloud_gt = cloud_gt[:,:3]
            cloud = np.append(cloud, cloud_gt, axis=0)
            cloud_shape = (cloud_01.shape[0], cloud_02.shape[0], cloud_gt.shape[0])

        # cloud = cloud[cloud[:,2] <= 1.0]

        viewer.update_cloud(cloud, cloud_shape)
        
        if not viewer.is_alive:
            break
            
        if (args.mode == "s"):
            break
        elif (args.mode == "fbf"):
            input()
        else:
            time.sleep(1.0 / 10)

if __name__ == "__main__":
    main()
