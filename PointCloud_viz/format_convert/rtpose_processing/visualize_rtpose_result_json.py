import numpy as np
import os
import os.path as osp
import argparse
import json
import matplotlib.pyplot as plt

joint_list = ['WaistLFront', 'WaistRFront', 'WaistLBack', 'WaistRBack', 'BackTop', 
              'Chest', 'BackLeft', 'BackRight', 'HeadTop', 'HeadFront', 
              'HeadSide', 'LShoulderBack', 'LShoulderTop', 'LElbowOut', 'LUArmHigh', 
              'LHandOut', 'LWristOut', 'LWristIn', 'RShoulderBack', 'RShoulderTop', 
              'RElbowOut', 'RUArmHigh', 'RHandOut', 'RWristOut', 'RWristIn',
              'LKneeOut', 'LThigh', 'LAnkleOut', 'LShin', 'LToeOut', 
              'LToeIn', 'RKneeOut', 'RThigh', 'RAnkleOut', 'RShin', 'RToeOut', 'RToeIn']

line_idx = np.array([[0,1],[1,2],[2,3],[0,4],[4,5],[5,6],[1,4],
                     [1,9],[4,12],[0,7],[9,12],[8,7],
                     [7,9],[9,10],[10,11],[7,12],[12,13],[13,14]])


def main():
    parser = argparse.ArgumentParser(
        prog="create_rtpose_train_json.py",
        description="Create RT-POSE'style train.json file from gt"
    )

    parser.add_argument(
        "-d", "--data_path",
        type=str,
        help="Path to result file"
    )

    parser.add_argument(
        "-gt", "--gt_path",
        type=str,
        help="Path to gt file"
    )

    args = parser.parse_args()

    with open(args.data_path, "r") as f:
        result = json.load(f)
    
    with open(args.gt_path, "r") as f:
        gt = json.load(f)

    seq_list = list(result.keys())
    fig = plt.figure()
    ax = plt.subplot(projection="3d")
    ax.view_init(elev=10, azim=-165)
    for seq in seq_list:
        seq_res = result[seq]
        seq_gt  = gt[seq]

        frame_list = list(seq_res.keys())
        for frame in frame_list:
            pose = np.asarray(seq_res[frame]["keypoints"])[:,1:-1]
            gt_frame = frame[:frame.rfind("_")]
            gt_pose = np.asarray(seq_gt[gt_frame][0]["pose"])
            print(pose.shape, gt_pose.shape)
            
            for id in line_idx:
                ax.plot([gt_pose[id[0],0], gt_pose[id[1],0]],
                        [gt_pose[id[0],1], gt_pose[id[1],1]],
                        [gt_pose[id[0],2], gt_pose[id[1],2]], c='blue')
                ax.plot([pose[id[0],0], pose[id[1],0]],
                        [pose[id[0],1], pose[id[1],1]],
                        [pose[id[0],2], pose[id[1],2]], c='red')


            ax.set_xlim(0,8)
            ax.set_ylim(-3,3)
            ax.set_zlim(-1,2)

            plt.draw()
            btnpress = plt.waitforbuttonpress(0.1) 
            if btnpress: 
                plt.waitforbuttonpress(-1)
            plt.cla()


if __name__ == "__main__":
    main()