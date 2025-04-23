"""
Clone from https://github.com/nghiaho12/rigid_transform_3D
with some adjustment for input feeding
"""

import numpy as np
import os
import argparse
from scipy.spatial.transform import Rotation

def main():
    parser = argparse.ArgumentParser(
        prog="centroid_calculate.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to picked list"
    )

    args = parser.parse_args()

    # # Load data
    # data = None
    # with open(args.file, "r") as f:
    #     data = f.read().split("\n")[:-1]
    
    # n_point = len(data)

    # pairs = [[(3, 1.5, -0.53),(-0.348034, 1.20692, -0.235197)],
    #         [(3.5, 1.9, -0.15),(-0.374312,1.635597,-0.835999)],
    #         [(2.7, 1.8, -0.48),(-0.725562,1.265908,-0.103301)],
    #         [(3.6, 1.5, -0.43),(-0.02597,1.38417,-0.787136)],
    #         [(3.2, 1.7, -0.22),(-0.469156,1.584865,-0.544484)]]
    
    pairs = [[(3.9, -0.18, 0.39),(3.5, 1.6, -0.46)],
            [(2.9, -0.89, 0.4),(2.9, 0.61, -0.44)],
            [(3, 0.3, 0.3),(2.5, 1.7, -0.51)],
            [(2.4, -0.092, 0.35),(2.1, 1.1, -0.48)],
            [(3.8, -0.65, 0.44),(3.6, 1.2, -0.41)]]

    n_point = len(pairs)

    radar_points = np.zeros((n_point, 3), dtype=np.float64)
    lidar_points = np.zeros((n_point, 3), dtype=np.float64)
    
    # for i, line in enumerate(data):
    #     p_txt = line.split(" --- ")
    #     radar_points[i] = np.array([np.float64(p) for p in p_txt[0].split(",")])
    #     lidar_points[i] = np.array([np.float64(p) for p in p_txt[1].split(",")])

    for i, pair in enumerate(pairs):
        radar_points[i] = np.array(pair[0])
        lidar_points[i] = np.array(pair[1])

    # print(radar_points)
    # print(lidar_points)
    # Calibrate
    ## Reshaping to 3xn_point
    radar_points = radar_points.T
    lidar_points = lidar_points.T

    ## Centering
    centroid_radar = np.mean(radar_points, axis=1)
    centroid_radar = centroid_radar.reshape(-1, 1)
    centroid_lidar = np.mean(lidar_points, axis=1)
    centroid_lidar = centroid_lidar.reshape(-1, 1)
    # print("Mean radar: ", mean_radar)
    # print("Mean lidar: ", mean_lidar)

    # centering matrices
    tile_radar = radar_points - centroid_radar
    tile_lidar = lidar_points - centroid_lidar
    # print("Centering radar: ", tile_radar)
    # print("Centering lidar: ", tile_lidar)

    ## Compute covariance matrix
    cov_mat = tile_radar @ tile_lidar.T
    # print(cov_mat.shape)

    ## matrix decomposition on rotation, scaling and rotation matrices
    [U, S, Vt] = np.linalg.svd(cov_mat)

    ## Rotation mat
    R = Vt.T @ U.T

    # special reflection case
    if np.linalg.det(R) < 0:
        print("det(R) < R, reflection detected!, correcting for it ...")
        Vt[2,:] *= -1
        R = Vt.T @ U.T
    
    # print(R.shape)

    # R = np.array([[0.8889418, 0.4564966, -0.0373286],
    #               [0.4568565, -0.8895395,  0.0012595],
    #               [-0.0326303, -0.0181735, -0.9993023]])
    t = -R @ centroid_radar + centroid_lidar

    print("R: ", R)
    print("R in quaternion: ", Rotation.from_matrix(R).as_quat())
    print("t: ", t.T)

if __name__ == "__main__":
    main()