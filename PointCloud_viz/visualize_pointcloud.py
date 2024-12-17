import numpy as np
import open3d as o3d
import os
import argparse
from pypcd4 import PointCloud
import time

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


        # # Assign callback functions
        # self.vis.register_key_callback(ord("t"), self.pause)
        # self.vis.register_key_callback(ord("s"), self.step)
        # self.vis.register_key_callback(ord("Q"), self.quit)

        # pcl object
        self.pcl = o3d.geometry.PointCloud()

        #
        # self.is_pause = False
        # self.is_step = False
        self.is_alive = True

    # def quit(self):
    #     self.vis.destroy_window()

    # def pause(self):
    #     self.is_pause = not self.is_pause

    # def step(self):
    #     self.is_step = True
    
    def update_cloud(self, cloud):    
        # Convert numpy array of 3D points to Vector3d of open3d
        self.pcl.points = o3d.utility.Vector3dVector(cloud)
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
        prog="visualize_pointcloud.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-d", "--data_dir",
        type=str,
        help="Path to pointcloud directory"
    )

    parser.add_argument(
        "-t", "--type",
        type=str,
        default="pcd"
        help="Point cloud type pcd/xyzdi"
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

    pcd_list = sorted(os.listdir(args.data_dir))
    
    # visualizer
    viewer = Viewer3D()
    for i, pcd_file in enumerate(pcd_list):
        if (i < args.start_frame):
            continue

        if (args.type == "pcd"):
            print("Timestamp: ", pcd_file[:-4]) 
            pcd = PointCloud.from_path(os.path.join(args.data_dir, pcd_file))
            cloud = pcd.numpy(("x", "y", "z"))
        else:
            print("Timestamp: ", pcd_file[:-6]) 
            pcd = np.loadtxt(os.path.join(args.data_dir, pcd_file))
            cloud = pcd[:,:3]
            
            # filter point cloud with range > 6 meter
            cloud = cloud[cloud[:, 0] <= 9.5]

        viewer.update_cloud(cloud)
        
        if not viewer.is_alive:
            break
            
        if (args.frame_by_frame_mode):
            input()
        else:
            time.sleep(1.0 / 15)

if __name__ == "__main__":
    main()