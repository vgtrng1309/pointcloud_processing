import numpy as np
import open3d as o3d
import os
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

def opti2sensor_gt(T_opti_box, T_box_sensor, points):
    points = np.hstack((points, np.ones((points.shape[0], 1)))).T
    
    # (4,4) @ (4,4) @ (4, N) => (4, N)
    return np.linalg.inv(T_box_sensor) @ np.linalg.inv(T_opti_box) @ points

def create_transformation_matrix(R, t):
    T_mat = np.eye(4, 4)
    R = np.asarray(R)
    t = np.asarray(t)

    R = Rotation.from_quat(R).as_matrix()
    T_mat[:3,:3] = R
    T_mat[:3,3] = t
    return T_mat

def load_extrinsic(path):
    data = None
    with open(path, "r") as f:
        data = json.load(f)
    return data

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
        default="pcd",
        help="Point cloud type pcd/xyzdi"
    )

    parser.add_argument(
        "-g", "--groundtruth",
        type=str,
        help="Path to groundtruth csv file"
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

    parser.add_argument(
        "--mapping_file",
        type=str,
        help="Path to timestamp mapping file"
    )


    args = parser.parse_args()
    
    T_box_bosch = create_transformation_matrix([0.509, -0.506, -0.487, -0.498], \
                                               [-0.076, -0.987, -0.037])

    T_box_lidar = create_transformation_matrix([-0.603, 0.365, 0.363, 0.609], \
                                               [0.080, -0.050, -0.080])

    if (args.type == "pcd"):
        T_box_sensor = T_box_lidar
    else:
        T_box_sensor = T_box_bosch

    # Get gt list
    if (args.groundtruth):
        gt_csv = None
        print(args.groundtruth)
        with open(args.groundtruth, "r") as f:
            gt_csv = f.read().split("\n")[1:-1] # Load, split by row and remove 1st and last row
        gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}

        # Get mapping data
        if (args.mapping_file):
            ts_map = np.loadtxt(args.mapping_file, dtype=str)

    # visualizer
    viewer = Viewer3D()
    if (args.groundtruth and args.data_dir):
        for i, bosch2gt in enumerate(ts_map):
            try:
                pcd_file = bosch2gt[0] + "." + args.type
                if (args.type == "pcd"):
                    print(i, " Timestamp: ", pcd_file[:-4]) 
                    pcd = PointCloud.from_path(os.path.join(args.data_dir, pcd_file))
                    cloud = pcd.numpy(("x", "y", "z"))
                else:
                    print(i, " Timestamp: ", pcd_file[:-6]) 
                    pcd = np.loadtxt(os.path.join(args.data_dir, pcd_file))
                    cloud = pcd[:,:3]
                    
                # filter point cloud with range > 6 meter and height > 2 meter
                cloud = cloud[cloud[:, 0] <= 6.0]
                cloud = cloud[cloud[:, 2] <= 2.0]
                cloud = cloud[cloud[:, 1] <= 2.5]
                cloud = cloud[cloud[:, 1] >= -2.5]

                # Append gt cloud
                gt = np.array(gt_csv[gt_idx[bosch2gt[1]]].split(",")[2:-1], np.float32)
                T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
                gt = gt[:-7]

                # point in world coordinate, reshape to (N, 3)
                points_world = gt.reshape(-1, 3)
                points_bosch = opti2sensor_gt(T_opti_box, T_box_sensor, points_world).T
                points_bosch = points_bosch[:,:3]

                # 
                cloud = np.append(cloud, points_bosch, axis=0)

                viewer.update_cloud(cloud)
                
                if not viewer.is_alive:
                    break
            except:
                print("Empty point")
                
            if (args.frame_by_frame_mode):
                input()
            else:
                time.sleep(1.0 / 30)

    else:
        if (args.data_dir):
            pcd_list = sorted(os.listdir(args.data_dir))
            for i, pcd_file in enumerate(pcd_list):
                if (i < args.start_frame):
                    continue

                if (args.type == "pcd"):
                    print(i, " Timestamp: ", pcd_file[:-4]) 
                    pcd = PointCloud.from_path(os.path.join(args.data_dir, pcd_file))
                    cloud = pcd.numpy(("x", "y", "z"))
                else:
                    print(i, " Timestamp: ", pcd_file[:-6]) 
                    pcd = np.loadtxt(os.path.join(args.data_dir, pcd_file))
                    cloud = pcd[:,:3]
                    
                # filter point cloud with range > 6 meter
                cloud = cloud[cloud[:, 0] <= 6.0]
                cloud = cloud[cloud[:, 2] <= 2.0]
                cloud = cloud[cloud[:, 1] <= 2.5]
                cloud = cloud[cloud[:, 1] >= -2.5]

                viewer.update_cloud(cloud)
                
                if not viewer.is_alive:
                    break
                    
                if (args.frame_by_frame_mode):
                    input()
                else:
                    time.sleep(1.0 / 30)
        else:
            for i, line in enumerate(gt_csv):
                try:
                    if (i < args.start_frame):
                        continue
                    line = line.split(",")
                    gt = np.array(line[2:-1], np.float32)
                    T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
                    gt = gt[:-7]
                    cloud = gt.reshape(-1, 3)
                    print(i, " Timestamp: ", line[1]) 

                    viewer.update_cloud(cloud)
                    if not viewer.is_alive:
                        break
                except:
                    print("Empty point")

                        
                if (args.frame_by_frame_mode):
                    input()
                else:
                    time.sleep(1.0 / 30)



if __name__ == "__main__":
    main()
