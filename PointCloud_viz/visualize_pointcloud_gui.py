"""
Source link: https://github.com/isl-org/Open3D/issues/5067
Adjusted by: trung.l.nguyen@tuni.fi
"""

import numpy as np
import open3d as o3d
import os
import argparse
import time

class Viewer3D(object):
    def __init__(self, title):
        self.CLOUD_NAME = 'cloud3d'
        self.first_cloud = True
        app = o3d.visualization.gui.Application.instance
        app.initialize()

        self.main_vis = o3d.visualization.O3DVisualizer(title)
        app.add_window(self.main_vis)

    def tick(self):
        app = o3d.visualization.gui.Application.instance
        tick_return = app.run_one_tick()
        if tick_return:
            self.main_vis.post_redraw()
        return tick_return

    def update_cloud(self, geometries):
        if self.first_cloud:
            def add_first_cloud():
                self.main_vis.add_geometry(self.CLOUD_NAME, geometries)
                self.main_vis.reset_camera_to_default()
                self.main_vis.setup_camera(60,
                                           [4, 2, 5],
                                           [0, 0, -1.5],
                                           [0, -1, 0])

            add_first_cloud()
            self.first_cloud = False
        else:
            def update_with_cloud():
                self.main_vis.remove_geometry(self.CLOUD_NAME)
                self.main_vis.add_geometry(self.CLOUD_NAME, geometries)

            update_with_cloud()

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

    args = parser.parse_args()

    data = None
    with open(args.file, "r") as f:
        data = f.read().split("\n")
        data = data[7:-1] # Remove header and last row

    viewer3d = Viewer3D("3D_view")
    pcd = o3d.geometry.PointCloud()

    for i, line in enumerate(data):
        if (i < args.start_frame):
            continue
        line = line.split(",")
        idx, time_offset = line[0], line[1]
        # print("Index: ", idx, " - Time: ", time_offset)
        # print(line[2:-8])
        try:
            body_markers = np.array(line[2:-8]).astype(np.float).reshape(-1, 3)
            rigid_body   = np.array(line[-8:-1]).astype(np.float)
            # print(body_markerss, body_markerss.shape)

            # Convert numpy array of 3D points to Vector3d of open3d
            pcd.points = o3d.utility.Vector3dVector(body_markers)

        except:
            print("Pass frame ", idx)

        viewer3d.update_cloud(pcd)
        viewer3d.tick()
        time.sleep(0.1)

        # if not viewer3d.poll_events():
        #     break

    viewer3d.destroy_window()


    while True:
        # Step 1) Perturb the cloud with a random walk to simulate an actual read
        # (based on https://github.com/isl-org/Open3D/blob/master/examples/python/visualization/multiple_windows.py)
        pts = numpy.asarray(cloud.points)
        magnitude = 0.005 * extent
        displacement = magnitude * (numpy.random.random_sample(pts.shape) -
                                    0.5)
        new_pts = pts + displacement
        cloud.points = o3d.utility.Vector3dVector(new_pts)

        # Step 2) Update the cloud and tick the GUI application
        viewer3d.update_cloud(cloud)
        viewer3d.tick()
        time.sleep(0.1)

if __name__ == "__main__":
    main()
