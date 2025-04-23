import numpy as np 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os
import os.path as osp
import argparse
import time
import json
from scipy.spatial.transform import Rotation

def normalize(v):
    return v / np.linalg.norm(v)

def rotate_vector(points):
    """
    Rotates vector v1 to align with vector v2 using Rodrigues' rotation formula.
    
    Args:
        v1: The initial vector (numpy array)
        v2: The target vector to align with (numpy array)
        
    Returns:
        rotated_v: The rotated vector (numpy array)
    """
    # Step 1: get norm vector
    A, B, C = points
    AB = B - A
    AC = C - A
    v2 = normalize(np.cross(AB, AC))
    v1 = np.array([0, 0, -1])
    
    # Step 2: Compute the axis of rotation (cross product of v1 and v2)
    axis = np.cross(v1, v2)
    
    # If the vectors are already aligned, return v1
    if np.linalg.norm(axis) < 1e-6:
        return v1
    
    # Normalize the axis
    axis = normalize(axis)
    
    # Step 3: Compute the angle of rotation (dot product between v1 and v2)
    angle = np.arccos(np.clip(np.dot(v1, v2), -1.0, 1.0))
    
    # Step 4: Rodrigues' rotation formula
    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    
    # Rotation matrix (3x3)
    R = np.array([
        [cos_angle + axis[0]**2 * (1 - cos_angle), axis[0] * axis[1] * (1 - cos_angle) - axis[2] * sin_angle, axis[0] * axis[2] * (1 - cos_angle) + axis[1] * sin_angle],
        [axis[1] * axis[0] * (1 - cos_angle) + axis[2] * sin_angle, cos_angle + axis[1]**2 * (1 - cos_angle), axis[1] * axis[2] * (1 - cos_angle) - axis[0] * sin_angle],
        [axis[2] * axis[0] * (1 - cos_angle) - axis[1] * sin_angle, axis[2] * axis[1] * (1 - cos_angle) + axis[0] * sin_angle, cos_angle + axis[2]**2 * (1 - cos_angle)]
    ])
    return R

def rotmat_to_euler(R):
    return Rotation.from_rotvec(R).as_euler('zyx', degrees=True)

def euler_to_rotmat(R):
    pass

def rotate_point(points, R):
    # 4x4
    R_mat = np.eye(4)
    R_mat[:3,:3] = R
    print(R_mat)
    
    # 4x3
    p_mat = np.ones((3, 4))
    p_mat[:,:3] = points
    p_mat = p_mat.T
    print(p_mat)

    # 4x3
    p_rot = np.matmul(R_mat, p_mat)
    return p_rot[:3,:].T

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_points.py",
        description="For calculating centroid of reflector on lidar"
    )

    # parser.add_argument(
    #     "-d", "--dir",
    #     type=str,
    #     help="Path to data dir",
    # )

    # parser.add_argument(
    #     "-s", "--start_frame",
    #     type=int,
    #     default=0,
    #     help="Starting frame"
    # )

    # parser.add_argument(
    #     "-m", "--mode",
    #     type=str,
    #     default="rt",
    #     help="Mode: rt-realtime, fbf-frame by frame, s-single, v-verification"
    # )

    # args = parser.parse_args()

    R = np.array([[0, -1, 0],
                  [0,  0, 1],
                  [-1, 0, 0]])

    points_06_03 = np.array([[-0.292822,0.77576,3.216472],
                        [-0.471563,0.776792,3.213037],
                        [-0.377317,1.085604,3.20402]])

    points_01_04 = np.array([[-0.073997,1.236251,4.194365],
                        [0.105793,1.230353,4.182975],
                        [0.007288,0.926631,4.198318]])

    points_08_04 = np.array([[-0.561179,1.214674,2.581825],
                        [-0.382185,1.204301,2.600529],
                        [-0.489325,0.903052,2.613246]])

    points = points_01_04
    R_mat = rotate_vector(points)
    points_rot = (R_mat.T @ points.T).T
    print(points_rot)

    roll_angle = np.arctan2(points_rot[0,1]-points_rot[1,1],
                            points_rot[0,0]-points_rot[1,0])
    roll_angle = (roll_angle - np.pi) if (roll_angle >= np.pi/2) else roll_angle
    roll_angle = (roll_angle + np.pi) % (2*np.pi) - np.pi
    print(roll_angle)
    R_roll = np.array([[np.cos(roll_angle), -np.sin(roll_angle), 0],
                       [np.sin(roll_angle), np.cos(roll_angle), 0],
                       [0, 0, 1]])

    print((R_roll.T @ points_rot.T).T)
    final_R = Rotation.from_matrix(np.matmul(np.matmul(R, R_mat), R_roll))

    print("final R: ", final_R.as_quat())

    # # 
    # fig = plt.figure(figsize=(12,10))
    # fig.tight_layout()

    # ax11 = fig.add_subplot(111, projection="3d")
    # ax11.view_init(elev=10, azim=-165)

    # # line_idx = np.array([[5,4],[4,9],[9,10],[10,11],[4,6],[6,7],[7,8],
    # #                      [9,1],[6,0],[1,0],[1,14],[14,15],[0,12],[12,13]])

    # ax11.cla()
    # ax11.scatter(points[:,0], points[:,1], points[:,2], s=5, c="red")
    # scale = 1.0
    # ax11.quiver(*point, *X*scale, color='r', label='X axis')
    # ax11.quiver(*point, *Y*scale, color='g', label='Y axis')
    # ax11.quiver(*point, *Z*scale, color='b', label='Z axis (normal)')

    
    # # ax11.scatter(gt[:,0], gt[:,1], gt[:,2], s=5, c='blue')
    # # for id in line_idx:
    # #     ax11.plot([gt[id[0],0], gt[id[1],0]],
    # #                 [gt[id[0],1], gt[id[1],1]],
    # #                 [gt[id[0],2], gt[id[1],2]], c='blue')

    # ax11.set_xlabel('X-metre')
    # ax11.set_ylabel('Y-metre')
    # ax11.set_zlabel('Z-metre')

    # ax11.set_xlim(-2, 2)
    # ax11.set_ylim(0, 3)
    # ax11.set_zlim(2.0, 4.5)

    # plt.show()

if __name__ == "__main__":
    main()
