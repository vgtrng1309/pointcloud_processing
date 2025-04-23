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
        

    def update_cloud(self, cloud, gt_size=0, pass_frame=0):
        # Update label
        # self.update_label(cloud)
        # Convert numpy array of 3D points to Vector3d of open3d

        # Convert numpy array of 3D points to Vector3d of open3d
        if self.mode != "v":
            self.pcl.points = o3d.utility.Vector3dVector(cloud)

            if ( gt_size != 0):
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

            
        # if (self.first_cloud):
        #     self.last_cloud = cloud
        #     if self.mode != "v":
        #         self.vis.add_geometry(self.pcl)
        #     self.first_cloud = False
        # elif (self.mode == "v"):
        #     distance = np.sqrt(np.sum((self.last_cloud - cloud)**2, axis=1))
        #     self.last_cloud = cloud

        #     distance = distance[distance >= 0.1*(pass_frame+1)]
        #     if distance.shape[0] > 0:
        #         print(distance)

        # if self.mode != "v":
        #     self.vis.update_geometry(self.pcl)
        #     if (self.mode != "s"):
        #         self.vis.poll_events()
        #         self.vis.update_renderer()
        #     else:
        #         self.vis.run()

        #     if (not self.vis.poll_events()):
        #         self.vis.destroy_window()
        #         self.is_alive = False

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

    # parser.add_argument(
    #     "-d", "--data_dirs",
    #     nargs="+",
    #     help="Path to pointcloud directories",
    # )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to data dir",
    )

    parser.add_argument(
        "-p", "--pcl_dir",
        type=str,
        default="",
        help="Path to pcl from root dir",
    )

    parser.add_argument(
        "-g", "--gt_dir",
        type=str,
        default="",
        help="Path to groundtruth csv file from root dir"
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

    parser.add_argument(
        "--mapping_file",
        type=str,
        help="Path to timestamp mapping file"
    )

    args = parser.parse_args()

    # "R": [0.509, -0.506, -0.487, -0.498],
    #         "t": [0.043, -0.952, -0.096],
    
    T_box_bosch = create_transformation_matrix([0.509, -0.506, -0.487, -0.498], \
                                               [-0.027, -1.122, -0.096])

    # T_box_lidar = create_transformation_matrix([-0.603, 0.366, 0.363, 0.610], \
    #                                            [0.140, -0.055, -0.059])
    T_box_lidar = create_transformation_matrix([-0.603, 0.366, 0.363, 0.610], \
                                               [0.050, -0.185, -0.059])
    
    # T_lidar_gt = create_transformation_matrix([-0.59617003,0.33754808,0.36829464,0.62849156], \
    #                                            [-0.54277663, 1.55323427, 3.12516637])

    T_box_ti = create_transformation_matrix([0.509, -0.506, -0.487, -0.498],
                                            [-0.041, -1.042, -0.120])

    tmp = os.listdir(osp.join(args.dir, args.pcl_dir))[0]
    type = "pcd" if tmp[-3:] == "pcd" else "xyzdi"

    if (type == "pcd"):
        T_box_sensor = T_box_lidar
    elif (type == "xyzdi"):
        # T_box_sensor = T_box_bosch
        T_box_sensor = T_box_ti

    # 
    pcl_dir = osp.join(args.dir, args.pcl_dir)

    # Get gt list
    if (args.gt_dir != ""):
        gt_dir = osp.join(args.dir, args.gt_dir)
        mapping_file = osp.join(args.dir, args.mapping_file)

        gt_csv = None
        print(gt_dir)
        with open(gt_dir, "r") as f:
            gt_csv = f.read().split("\n")[1:-1] # Load, split by row and remove 1st and last row
        gt_idx = {str(line.split(",")[1]): int(line.split(",")[0]) for line in gt_csv}

        # Get mapping data
        if (mapping_file):
            ts_map = np.loadtxt(mapping_file, dtype=str)

    # visualizer
    viewer = Viewer3D(args.mode)
    if (args.gt_dir != "" and args.pcl_dir != ""):

        for j in range(ts_map[args.start_frame:].shape[0]):
            if (args.mode != "s"):
                i = j + args.start_frame
            else:
                i = args.start_frame
            bosch2gt = ts_map[i]

            pcd_file = bosch2gt[0] + "." + type
            if (type == "pcd"):
                print(i, " Timestamp: ", pcd_file[:-4]) 
                pcd = PointCloud.from_path(os.path.join(pcl_dir, pcd_file))
                cloud = pcd.numpy(("x", "y", "z"))

                cloud = cloud[cloud[:, 2] > -1.75]
                cloud = cloud[cloud[:, 2] < 1.0]
                cloud = cloud[cloud[:, 0] > 0.0]
                cloud = cloud[cloud[:, 0] < 4.5]

            else:
                print(i, " Timestamp: ", pcd_file[:-6]) 
                pcd = np.loadtxt(os.path.join(pcl_dir, pcd_file))
                cloud = pcd

                if ("radarpcl" in pcl_dir):
                    cloud[:,[0, 1]] = cloud[:,[1, 0]]


            # filter point cloud with range > 6 meter and height > 2 meter
            # cloud = cloud[cloud[:, 0] <= 7.0]
            # cloud = cloud[cloud[:, 0] >= 0.5]

            # cloud = cloud[cloud[:, 2] <= 2.0]
            # cloud = cloud[cloud[:, 1] <= 2.5]
            # cloud = cloud[cloud[:, 1] >= -2.5]

            # filter by doppler and intensity
            # if (type == "xyzdi"):
            #     cloud = cloud[cloud[:, 4] >= 25.0]
            #     cloud = cloud[abs(cloud[:, 3]) >= 0.05]
            # cloud = cloud[:64]

            # Append gt cloud
            gt = gt_csv[gt_idx[bosch2gt[1]]].split(",")[2:-1]
            if ("" in gt):
                continue
            gt = np.array(gt, np.float32)
            T_opti_box = create_transformation_matrix(gt[-7:-3], gt[-3:])
            gt = gt[:-7]

            # point in world coordinate, reshape to (N, 3)
            points_world = gt.reshape(-1, 3)
            points_gt = opti2sensor_gt(T_opti_box, T_box_sensor, points_world).T
            # points_gt = opti2sensor_gt(np.eye(4), T_lidar_gt, points_world).T
            points_gt = points_gt[:,:3]

            # 
            if (type == "xyzdi"):
                cloud = cloud[:,:3]

            cloud = np.append(cloud, points_gt, axis=0)

            viewer.update_cloud(cloud, points_gt.shape[0])
            
            if not viewer.is_alive:
                break
                
            if (args.mode == "fbf"):
                input()
            else:
                time.sleep(1.0 / 10)
    else:
        if (args.pcl_dir != ""):
            pcd_list = sorted(os.listdir(pcl_dir))
            for i, pcd_file in enumerate(pcd_list):
                if (i < args.start_frame):
                    continue

                if (type == "pcd"):
                    print(i, " Timestamp: ", pcd_file[:-4]) 
                    pcd = PointCloud.from_path(os.path.join(pcl_dir, pcd_file))
                    cloud = pcd.numpy(("x", "y", "z"))
                else:
                    print(i, " Timestamp: ", pcd_file[:-6]) 
                    pcd = np.loadtxt(os.path.join(pcl_dir, pcd_file))
                    cloud = pcd

                    if ("radarpcl" in pcl_dir):
                        cloud[:,[0, 1]] = cloud[:,[1, 0]]

                    
                # filter point cloud within range [0.5, 8.0] meter
                cloud = cloud[cloud[:, 0] <= 8.0]
                cloud = cloud[cloud[:, 0] >= 0.5]

                # cloud = cloud[cloud[:, 2] <= 2.0]
                # cloud = cloud[cloud[:, 1] <= 1.5]
                # cloud = cloud[cloud[:, 1] >= -1.5]


                # # filter by doppler and intensity
                # if (type == "xyzdi"):
                #     cloud = cloud[cloud[:, 4] >= 25.0]
                #     cloud = cloud[abs(cloud[:, 3]) >= 0.1]
                # cloud = cloud[:64]

                viewer.update_cloud(cloud[:,:3])
                
                if not viewer.is_alive:
                    break
                    
                if (args.mode == "s"):
                    break
                elif (args.mode == "fbf"):
                    input()
                else:
                    time.sleep(1.0 / 10)
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

                        
                if (args.mode == "s"):
                    break
                elif (args.mode == "fbf"):
                    input()
                else:
                    time.sleep(1.0 / 10)



if __name__ == "__main__":
    main()
