import numpy as np
import open3d as o3d
import os
import os.path as osp
import argparse
from pypcd4 import PointCloud
import time
import json
from scipy.spatial.transform import Rotation

class Viewer3D(object):
    def __init__(self):
        self.first_cloud = True
        # self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=True)
        
        # Set option. Call only after creating visualizer window.
        opt = self.vis.get_render_option()
        opt.background_color = np.asarray([0.0, 0.5, 0.5])
        
        # Draw axis
        mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.vis.add_geometry(mesh)

        # pcl object
        self.pcl = o3d.geometry.PointCloud()

        self.is_alive = True

    def update_cloud(self, cloud, gt_size=0):
        # Convert numpy array of 3D points to Vector3d of open3d
        self.pcl.points = o3d.utility.Vector3dVector(cloud)

        if (gt_size != 0):
            np_color = np.zeros((cloud.shape[0],3))
            np_color[:-gt_size,0] = 1.0
            np_color[-gt_size:,2] = 1.0
            
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

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_mars_data.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to data dir"
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
        "-m", "--frame_by_frame_mode",
        action="store_true",
        help="Load frame by frame"
    )

    args = parser.parse_args()

    # seq name
    seq = args.dir[args.dir.rfind("/")+1:]
    ft_path_tail = f"radarpcl_mars/featuremap_{seq}_{args.type}.npy"
    lb_path_tail = f"radarpcl_mars/labels_{seq}_{args.type}.npy"

    # visualizer
    viewer = Viewer3D()
    fts = np.load(osp.join(args.dir, ft_path_tail)).reshape(-1, 256, 5)
    lbs = np.load(osp.join(args.dir, lb_path_tail)).reshape(-1, 15, 3)
    i = 0

    print(fts.shape, lbs.shape) 
    for ft, lb in zip(fts, lbs):
        if (i < args.start_frame):
            i += 1
            continue

        cloud = ft[:,:3]
        points_gt = lb

        print(cloud.shape, lb.shape)

        cloud = np.append(cloud, points_gt, axis=0)

        viewer.update_cloud(cloud, points_gt.shape[0])
        
        if not viewer.is_alive:
            break
        # except:
        #     print("Empty point")
            
        if (args.frame_by_frame_mode):
            input()
        else:
            time.sleep(1.0 / 20)

        i += 1


if __name__ == "__main__":
    main()
