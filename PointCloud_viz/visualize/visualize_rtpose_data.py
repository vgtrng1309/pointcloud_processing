import numpy as np
import os
import os.path as osp
import argparse
import json
import matplotlib.pyplot as plt
import time

rtpose_gt_path_tail = "gt/Train.json"
rtpose_hm_path_tail = "radar/npy_DZYX_real"

x_min       = 0     #(m)
x_max       = 15.1836  #(m) x_max = range size +x_per_bin; 15.1836

y_min       = -14.953     #(m) # y_max = np.sin(max_azimuth)*max_range = np.sin(80)*15.1836
y_max       = 14.953   #(m)   => y_res = 14.953*2 / 128 = 0.233640625

z_min       = -5.193    #(m)      z_max = np.sin(max_elevation)*max_range = np.sin(20)*15.1836
z_max       = 5.193   #(m)     => z_res = 5.193*2 / 32 = 0.3245625

def main():
    parser = argparse.ArgumentParser(
        prog="visualize_rtpose.py",
        description="Visualize RT-Pose gt for verification"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to data dir"
    )

    parser.add_argument(
        "-st", "--start_frame",
        type=int,
        default=0,
        help="Skip to frame st"
    )

    parser.add_argument(
        "-m", "--mode",
        type=str,
        default="both",
        help="Visualization mode ground truth (gt), heat map (hm) or both"
    )

    args = parser.parse_args()

    gt_json = None
    with open(osp.join(args.dir, rtpose_gt_path_tail), "r") as f:
        gt_json = json.load(f)

    # Remove seq name
    gt_json = gt_json[list(gt_json.keys())[0]]

    # Frame list
    fig = plt.figure()
    fig.tight_layout()
    ax1 = plt.subplot(221, projection="3d")
    ax1.view_init(elev=10, azim=-165)

    ax2 = plt.subplot(222)
    ax3 = plt.subplot(223)
    ax4 = plt.subplot(224)
    ax2_img, ax3_img, ax4_img = None, None, None
    first_frame = True
    plt.draw()

    doppler = 0

    frame_list = list(gt_json.keys())
    for i, frame in enumerate(frame_list):
        if (i < args.start_frame):
            continue

        st = time.time()
        # Show gt
        ax1.cla()
        frame_gt = gt_json[frame]
        for j, gt in enumerate(frame_gt):
            points_gt = np.asarray(gt["pose"])
            # points_gt = np.delete(points_gt, 12, ax1is=0)
            ax1.scatter(points_gt[:,0], points_gt[:,1], points_gt[:,2], s=5, c='blue')
        
        ax1.set_xlabel('X-metre')
        ax1.set_ylabel('Y-metre')
        ax1.set_zlabel('Z-metre')

        ax1.set_xlim(0, 10)
        ax1.set_ylim(-3, 3)
        ax1.set_zlim(-2.5, 2.5)

        # Show radar data
        # ax2.cla()
        # ax3.cla()
        # ax4.cla()
        hm_path = osp.join(args.dir, rtpose_hm_path_tail, frame+".npy")
        hm = np.load(hm_path)[doppler, :, :, :].astype(np.float32) # ZYX
        hm = 2**hm

        # X-Y view
        xy_view = np.transpose(np.mean(hm, axis=0), (1, 0))
        xy_view[xy_view < 1] = 1
        xy_view = np.log2(xy_view)
        print(np.min(xy_view), np.max(xy_view))
        # xy_view[xy_view < 1.] = 1.

        # Y-Z view
        zy_view = np.mean(hm,axis=2)
        zy_view[zy_view<=1] = 1
        zy_view = np.log2(zy_view)

        # Z-X view
        zx_view = np.mean(hm,axis=1)
        zx_view[zx_view<=1] = 1
        zx_view = np.log2(zx_view)

        if (first_frame):
            ax2_img = ax2.imshow(xy_view[::-1, :], cmap='jet',vmin=0,vmax=20)
            ax2.set_xlabel('Y-metre')
            ax2.set_ylabel('X-metre')
            ax2.set_aspect(0.5)

            x_plot_loc = np.linspace(0, xy_view.shape[1], 11).astype(int)
            y_plot_loc = np.linspace(0, xy_view.shape[0], 11).astype(int)
            ax2.set_xticks(x_plot_loc, -( x_plot_loc/xy_view.shape[1] * (y_max - y_min) + y_min).astype(int))
            ax2.set_yticks(y_plot_loc, (-y_plot_loc/xy_view.shape[0] * (x_max - x_min) + x_max).astype(int))

            ax3_img = ax3.imshow(zy_view[::-1, :],cmap='jet',vmin=0,vmax=20)
            #fig.colorbar(im01, orientation='vertical')

            ax3.set_title('Y-Z')
            ax3.set_xlabel('axis-Y (m)')
            ax3.set_ylabel('axis-Z (m)')
            ax3.set_aspect(2)
            x_plot_loc = np.linspace(0, zy_view.shape[1], 11).astype(int)
            y_plot_loc = np.linspace(0, zy_view.shape[0], 11).astype(int)
            ax3.set_xticks(x_plot_loc, -( x_plot_loc/zy_view.shape[1] * (y_max - y_min) + y_min).astype(int))
            ax3.set_yticks(y_plot_loc, (-y_plot_loc/zy_view.shape[0] * (z_max - z_min) + z_max).astype(int))

            ax4_img = ax4.imshow(zx_view[::-1, :],cmap='jet',vmin=0,vmax=20)
            #fig.colorbar(im10, orientation='vertical')

            ax4.set_title('X-Z')
            ax4.set_xlabel('axis-X (m)')
            ax4.set_ylabel('axis-Z (m)')
            ax4.set_aspect(4)
            x_plot_loc = np.linspace(0, zx_view.shape[1], 11).astype(int)
            y_plot_loc = np.linspace(0, zx_view.shape[0], 11).astype(int)
            ax4.set_xticks(x_plot_loc, ( x_plot_loc/zx_view.shape[1] * (x_max - x_min) + x_min).astype(int))
            ax4.set_yticks(y_plot_loc, (-y_plot_loc/zx_view.shape[0] * (z_max - z_min) + z_max).astype(int))

            first_frame = False
        else:
            ax2_img.set_data(xy_view[::-1, :])
            ax3_img.set_data(zy_view[::-1, :])
            ax4_img.set_data(zx_view[::-1, :])

        print(time.time() - st)
        plt.pause(0.001)

if __name__ == "__main__":
    main()