import numpy as np
import open3d as o3d
import os
import argparse

class Viewer3D(object):
    def __init__(self):
        self.first_cloud = True
        # self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(visible=True)
        
        # Set option. Call only after creating visualizer window.
        opt = self.vis.get_render_option()
        opt.show_coordinate_frame = True
        opt.background_color = np.asarray([0.5, 0.5, 0.5])
        
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
        prog="visualize_pointcloud_csv.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to csv file"
    )

    parser.add_argument(
        "-s", "--start_frame",
        type=int,
        default=0,
        help="Starting frame"
    )

    parser.add_argument(
        "-m", "--frame_by_frame_mode",
        action="store_false",
        help="Load frame by frame"
    )

    args = parser.parse_args()

    data = None
    with open(args.file, "r") as f:
        data = f.read().split("\n")
        data = data[7:-1] # Remove header and last row
    
    # visualizer
    viewer = Viewer3D()
    for i, line in enumerate(data):
        if (i < args.start_frame):
            continue
        
        line = line.split(",")
        idx, time_offset = line[0], line[1]
        print("Index: ", idx, " - Time: ", time_offset)
        try:
            body_markers = np.array(line[2:-8]).astype(np.float).reshape(-1, 3)
            rigid_body   = np.array(line[-8:-1]).astype(np.float)

            viewer.update_cloud(body_markers)
        except:
            print("Pass frame ", idx)
        
        if not viewer.is_alive:
            break
            
        if (args.frame_by_frame_mode):
            input()

if __name__ == "__main__":
    main()