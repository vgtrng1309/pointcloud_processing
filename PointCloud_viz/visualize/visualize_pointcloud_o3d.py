import numpy as np
import open3d as o3d
import os
import os.path as osp
import argparse
import time
from pypcd4 import PointCloud

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
            # if (mode == "s"):
            #     opt.show_coordinate_frame = True
            opt.background_color = np.asarray([0.5, 0.5, 0.5])

            # pcl object
            self.pcl = o3d.geometry.PointCloud()
            
            # Draw axis
            if (mode != "s"):
                mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
                self.vis.add_geometry(mesh)

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
        

    def update_cloud(self, cloud, pass_frame=0):
        # Update label
        # self.update_label(cloud)
        
        # Convert numpy array of 3D points to Vector3d of open3d
        if self.mode != "v":
            self.pcl.points = o3d.utility.Vector3dVector(cloud)
            
        if (self.first_cloud):
            self.last_cloud = cloud
            if self.mode != "v":
                self.vis.add_geometry(self.pcl)
            self.first_cloud = False
        elif (self.mode == "v"):
            distance = np.sqrt(np.sum((self.last_cloud - cloud)**2, axis=1))
            self.last_cloud = cloud

            distance = distance[distance >= 0.1*(pass_frame+1)]
            if distance.shape[0] > 0:
                print(distance)

        if self.mode != "v":
            self.vis.update_geometry(self.pcl)
            if (self.mode != "s"):
                self.vis.poll_events()
                self.vis.update_renderer()
            else:
                self.vis.run()

            if (not self.vis.poll_events()):
                self.vis.destroy_window()
                self.is_alive = False

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_pointcloud_csv.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to point cloud dir"
    )

    parser.add_argument(
        "-s", "--start_frame",
        type=int,
        default=0,
        help="Starting frame"
    )

    parser.add_argument(
        "--delimiter",
        type=str,
        default=" ",
        help="Delimiter used in csv file"
    )

    parser.add_argument(
        "-m", "--mode",
        type=str,
        default="rt",
        help="Mode: rt-realtime, fbf-frame by frame, s-single, v-verification"
    )

    args = parser.parse_args()
    dil = args.delimiter
    viewer = Viewer3D(args.mode)
    pcl_type = args.dir[args.dir.rfind("/")+1:]

    if (pcl_type == "lidar"):
        pcl_list = sorted(os.listdir(args.dir))
        for i, pcl_file in enumerate(pcl_list[args.start_frame:]):
            print("Showing: frame_", i+args.start_frame)
            pcd = PointCloud.from_path(osp.join(args.dir, pcl_file))
            points = pcd.numpy(("x", "y", "z"))
            points = points[points[:, 2] > -1.0]
            points = points[points[:, 2] < 0.0]
            points = points[points[:, 0] >= 0.0]
            points = points[points[:, 0] <= 5.0]
            points = points[points[:, 1] >= -3.0]
            points = points[points[:, 1] <= 3.0]


            viewer.update_cloud(points)
            #except:
            #    pass_frame += 1
                # print("Pass frame ", idx)
            
            if not viewer.is_alive:
                break
                
            if (args.mode == "s"):
                break
            elif (args.mode == "fbf"):
                input()
            else:
                time.sleep(1.0 / 10)

    else:
        data_dir = args.dir
        pcl_list = sorted(os.listdir(data_dir))
        for i, pcl in enumerate(pcl_list):
            if (i < args.start_frame):
                continue

            data = None
            print("Showing: ", pcl)
            with open(os.path.join(data_dir, pcl), "r") as f:
                data = f.read().split("\n")[1:-1] # remove header and empty line at the end
            
            # print("Index: ", i+1, " - Time: ", pcl.replace(".csv", ""))
            points = np.array([np.array(line.split(dil), dtype=np.float64) for line in data])

            # Current order: azimuth-range-elevation convert to range-azimuth-elevation
            #                correspondent to x-y-z
            points[:,[0, 1]] = points[:,[1, 0]]
            # points[:,0] *= -1

            # filter
            # points = points[points[:,0] >= q2.0]

            viewer.update_cloud(points[:,:3])

            if not viewer.is_alive:
                break
                
            if (args.mode == "s"):
                break
            if (args.mode == "fbf"):
                input()
            else:
                time.sleep(1.0 / 10)



if __name__ == "__main__":
    main()
