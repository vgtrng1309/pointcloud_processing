import numpy as np
import os
import os.path as osp
import json
from scipy.spatial.transform import Rotation
import argparse

j_order = ["Pelvic", "WaistRBack", "RKneeOut", "RAnkleOut", "WaistLBack", "LKneeOut", 
             "LAnkleOut", "BackTop", "HeadTop", "RShoulderTop","RElbowOut", 
             "RWristOut", "LShoulderTop", "LElbowOut", "LWristOut"]

left_arm_idx = [4, 12, 13]
right_arm_idx = [1, 9, 10]
left_leg_idx = [12, 4, 5]
right_leg_idx = [9, 1, 2]

left_akl_idx = [12, 13, 14]
right_akl_idx = [9, 10, 11]
left_kne_idx = [4, 5, 6]
right_kne_idx = [1, 2, 3]


def MPJPE(pred, gt):
    # input shape: (batch_size, 15, 3)
    mpjpe_j = np.linalg.norm(pred - gt, axis=2)
    mpjpe = np.mean(mpjpe_j)
    mpjpe = np.append(mpjpe, np.mean(mpjpe_j, axis=0))
    return mpjpe

def AngleErr(pred, gt):
    def cal_joint_angle(joints):
        # Get dot product => cosine angle
        P1, P2, P3 = joints[:,0,:], joints[:,1,:], joints[:,2,:]
        P21 = P1 - P2
        P23 = P3 - P2
        
        dot_product = np.einsum('ij,ij->i', P21, P23)
        cosine_angle = dot_product / (np.linalg.norm(P21, axis=1)*np.linalg.norm(P23, axis=1))
        angle = np.arccos(cosine_angle)
        return np.expand_dims(angle, axis=1)

    def cal_batch_angle(batch):
        return np.hstack((cal_joint_angle(batch[:,left_arm_idx,:]),
                        cal_joint_angle(batch[:,right_arm_idx,:]),
                        cal_joint_angle(batch[:,left_leg_idx,:]),
                        cal_joint_angle(batch[:,right_leg_idx,:]),
                        cal_joint_angle(batch[:,left_akl_idx,:]),
                        cal_joint_angle(batch[:,right_akl_idx,:]),
                        cal_joint_angle(batch[:,left_kne_idx,:]),
                        cal_joint_angle(batch[:,right_kne_idx,:])))

    pred_angles = cal_batch_angle(pred)
    gt_angles = cal_batch_angle(gt)

    angles_diff = ((pred_angles - gt_angles) + np.pi) % (2*np.pi) - np.pi
    angles_score = np.mean(np.abs(angles_diff))
    angles_score = np.append(angles_score, np.mean(np.abs(angles_diff), axis=0))
    return angles_score * 180.0 / np.pi

def main():
    parser = argparse.ArgumentParser(
        prog="evaluate_hpe3d.py",
        description="Evaluate estimation of the 3D HPE"
    )

    parser.add_argument(
        "-p", "--pred",
        type=str,
        default="",
        help="Path to model HPE output"
    )

    parser.add_argument(
        "-g", "--gt",
        type=str,
        default="",
        help="Path to HPE ground truth"
    )

    parser.add_argument(
        "-o", "--output",
        type=str,
        default="",
        help="Path to save evaluation result"
    )

    parser.add_argument(
        "--model",
        type=str,
        default="PointMLP",
        help="Model type"
    )

    parser.add_argument(
        "--date",
        type=str,
        default="",
        help="Date record"
    )

    args = parser.parse_args()

    # Load prediction and ground truth
    pred = np.load(args.pred)
    if (len(pred.shape) != 3):
        pred = pred.reshape(-1, 15, 3)
    gt = np.load(args.gt)
    if (len(gt.shape) != 3):
        gt = gt.reshape(-1, 15, 3)
    print(pred.shape, gt.shape) 

    # 
    mpjpe = MPJPE(pred, gt)

    #
    angle_err = AngleErr(pred, gt)

    print(mpjpe)
    print(angle_err)

    if (args.output != ""):
        output_file = f"{args.model}_{args.date}_eval.json"
        result_json = {
            "Model_name": args.model,
            "Date": args.date,
            "MPJPE": dict([("MPJPE_all", mpjpe[0])] + \
                          [("MPJPE_"+j_order[i], mpjpe[i+1]) for i in range(mpjpe.shape[0]-1)]),
            "AngleErr": {
                "AngErr_all": angle_err[0],
                "AngErr_left_shoulder": angle_err[1], 
                "AngErr_right_shoulder": angle_err[2], 
                "AngErr_left_thigh": angle_err[3], 
                "AngErr_right_thigh": angle_err[4], 
                "AngErr_left_elbow": angle_err[5], 
                "AngErr_right_elbow": angle_err[6], 
                "AngErr_left_knee": angle_err[7], 
                "AngErr_right_knee": angle_err[8], 
            }
        }
        with open(osp.join(args.output, output_file), "w") as f:
            json.dump(result_json, f, indent=2)

if __name__ == "__main__":
    main()