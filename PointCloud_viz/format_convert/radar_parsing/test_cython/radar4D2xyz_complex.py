import numpy as np
import math
import re
import os
import os.path as osp
import argparse
import itertools
import time

#for plot
import matplotlib.pyplot as plt
from matplotlib.offsetbox import OffsetImage, AnnotationBbox

from radar4D2xyz_complex_cython import DRAE_to_DZYX

fig,axs = plt.subplots(2,2,figsize=(12,10))
first_frame = True
im00, im01, im10, im11 = None, None, None, None
path_save_dzyx_c_npy_tail =  'radar/npy_DZYX_real/'
path_save_fig_tail = 'radar/figure/'
path_folder_img = 'images/'
path_folder_tail = 'radar/mat/'

x_min       = 0     #(m)
x_max       = 15.1836  #(m) x_max = range size +x_per_bin; 15.1836

y_min       = -14.953     #(m) # y_max = np.sin(max_azimuth)*max_range = np.sin(80)*15.1836
y_max       = 14.953   #(m)   => y_res = 14.953*2 / 128 = 0.233640625

z_min       = -5.193    #(m)      z_max = np.sin(max_elevation)*max_range = np.sin(20)*15.1836
z_max       = 5.193   #(m)     => z_res = 5.193*2 / 32 = 0.3245625

def viz_YX_YZ_ZX_CAM(arrZYX,path_save,xyz_bound,path_img,anim=False):
    # print(path_save)
    if (anim):
        global first_frame, im00, im01, im11, im10 
        global fig, axs
    else:
        fig,axs = plt.subplots(2,2,figsize=(12,10))

    x_min, x_max, y_min, y_max, z_min, z_max = xyz_bound
    # fig,axs = plt.subplots(2,2,figsize=(12,10))
    
    #do flip for viz
    # arrZYX = np.abs(arrZYX)
    # arrZYX_flip = np.flip(arrZYX,axis=1)
    # arrZYX[arrZYX<1.] = 1.
    # # print(
    # print(np.min(arrZYX), np.max(arrZYX))
    # arrZYX = np.log2(arrZYX).astype(np.float16)
    # print(np.min(arrZYX), np.max(arrZYX))
    # arrZYX = 2**arrZYX.astype(np.float32)
    # print(np.min(arrZYX), np.max(arrZYX))
    # BEV Y-X plt
    showXY = np.transpose(np.mean(arrZYX,axis=0),(1,0))
    # showXY = np.transpose(arrZYX[:,16,:],(1,0))
    showXY[showXY<1.] = 1.
    # print(
    showXY = np.log2(showXY).astype(np.float16)
    # print(np.min(showXY), np.max(showXY))

    if first_frame or not anim:
        im00 = axs[0, 1].imshow(showXY[::-1, :],cmap='jet',vmin=0,vmax=20)
        fig.colorbar(im00, orientation='vertical')
        axs[0, 1].set_title('Y-X (BEV)')
        axs[0, 1].set_xlabel('axis-Y (m)')
        axs[0, 1].set_ylabel('axis-X (m)')
        axs[0, 1].set_aspect(0.5)
        x_plot_loc = np.linspace(0, showXY.shape[1], 11).astype(int)
        y_plot_loc = np.linspace(0, showXY.shape[0], 11).astype(int)
        axs[0, 1].set_xticks(x_plot_loc, -( x_plot_loc/showXY.shape[1] * (y_max - y_min) + y_min).astype(int))
        axs[0, 1].set_yticks(y_plot_loc, (-y_plot_loc/showXY.shape[0] * (x_max - x_min) + x_max).astype(int))
    else:
        im00.set_data(showXY[::-1, :])

    # Y-Z plt
    showZY = np.mean(arrZYX,axis=2)
    showZY[showZY<=1] = 1
    showZY = np.log2(showZY)
    if first_frame or not anim:
        im01 = axs[1, 1].imshow(showZY[::-1, :],cmap='jet',vmin=0,vmax=20)
        #fig.colorbar(im01, orientation='vertical')

        axs[1, 1].set_title('Y-Z')
        axs[1, 1].set_xlabel('axis-Y (m)')
        axs[1, 1].set_ylabel('axis-Z (m)')
        axs[1,1].set_aspect(2)
        x_plot_loc = np.linspace(0, showZY.shape[1], 11).astype(int)
        y_plot_loc = np.linspace(0, showZY.shape[0], 11).astype(int)
        axs[1, 1].set_xticks(x_plot_loc, -( x_plot_loc/showZY.shape[1] * (y_max - y_min) + y_min).astype(int))
        axs[1, 1].set_yticks(y_plot_loc, (-y_plot_loc/showZY.shape[0] * (z_max - z_min) + z_max).astype(int))
    else:
        im01.set_data(showZY[::-1, :])

    # Z-X plt
    showZX = np.mean(arrZYX,axis=1)
    showZX[showZX<=1] = 1
    showZX = np.log2(showZX)
    if first_frame or not anim:
        im10 = axs[1, 0].imshow(showZX[::-1, :],cmap='jet',vmin=0,vmax=20)
        #fig.colorbar(im10, orientation='vertical')

        axs[1, 0].set_title('X-Z')
        axs[1, 0].set_xlabel('axis-X (m)')
        axs[1, 0].set_ylabel('axis-Z (m)')
        axs[1,0].set_aspect(4)
        x_plot_loc = np.linspace(0, showZX.shape[1], 11).astype(int)
        y_plot_loc = np.linspace(0, showZX.shape[0], 11).astype(int)
        axs[1, 0].set_xticks(x_plot_loc, ( x_plot_loc/showZX.shape[1] * (x_max - x_min) + x_min).astype(int))
        axs[1, 0].set_yticks(y_plot_loc, (-y_plot_loc/showZX.shape[0] * (z_max - z_min) + z_max).astype(int))
    else:
        im10.set_data(showZX[::-1, :])    

    # # camera pic
    if (path_img != ""):
        if first_frame or not anim:
            im11 = axs[0, 0].imshow(plt.imread(path_img))
            # imagebox = OffsetImage(im11, zoom=0.25)
            # imagebox.image.axes = axs[0,0]
            # ab = AnnotationBbox(imagebox, (0.42, 0.5), xycoords='axes fraction',
            #                     bboxprops={'lw':0})
            # axs[0, 0].add_artist(ab)
            axs[0, 0].set_xticks([])
            axs[0, 0].set_yticks([])
            axs[0, 0].grid("False")
            axs[0, 0].spines['top'].set_visible(False)
            axs[0, 0].spines['right'].set_visible(False)
            axs[0, 0].spines['bottom'].set_visible(False)
            axs[0, 0].spines['left'].set_visible(False)
        else:
            im11.set_data(plt.imread(path_img))

    if first_frame:
        first_frame = False
    if (not anim):
        plt.savefig(path_save)
        plt.close()
    else:
        plt.draw()
        plt.pause(0.001)
    return True

def parse_args():
    parser = argparse.ArgumentParser(description="4D radar tensor trans to cartesian coordinate")
    parser.add_argument("-d", "--dir", help="the file path of dataset")
    parser.add_argument("-st", "--start_frame", type=int, 
                        default=0, help="the file path of dataset")
    parser.add_argument("-viz", action="store_true",
                        help="Decide to visualize or save the heatmap")
    parser.add_argument("-sviz", action="store_true",
                        help="Decide to save the visualization of the heatmap")
    parser.add_argument("-shm", action="store_true",
                        help="Decide to save the heatmap")
    args = parser.parse_args()

    return args

def main():
    args = parse_args()
    wait_time = 20.0

    path_dataset = args.dir
    file_list = sorted(os.listdir(os.path.join(path_dataset, path_folder_tail)))

    if (not os.path.exists(os.path.join(path_dataset,path_save_dzyx_c_npy_tail))):
        os.makedirs(os.path.join(path_dataset,path_save_dzyx_c_npy_tail))

    ts_map = None
    has_img = False
    path_mapping_file = 'calib/radarpcl2images_mapping.txt'
    if (os.path.exists(os.path.join(path_dataset,path_mapping_file))):
        has_img = True
        ts_map = np.loadtxt(os.path.join(path_dataset,path_mapping_file), dtype=str)    
    else:
        path_mapping_file = 'calib/radarpcl2gt_mapping.txt'
        ts_map = np.loadtxt(os.path.join(path_dataset,path_mapping_file), dtype=str)    
    
    if (not osp.exists(osp.join(args.dir, path_save_dzyx_c_npy_tail))):
        os.mkdir(osp.join(args.dir, path_save_dzyx_c_npy_tail))

    curr_idx = args.start_frame + 1
    st = time.time()
    while (True):
        if (time.time() - st > wait_time):
            break
        
        file = os.path.join(path_dataset, path_folder_tail, 
                            "4dTensor-Frame_"+str(curr_idx).zfill(4)+".bin")
        if (not osp.exists(file)):
            continue

        curr_idx += 1
        st = time.time()
        i = int(file[file.rfind("_")+1:].replace(".bin","")) - 1
        if (i < args.start_frame):
            continue

        path_img = ""
        if (has_img):
            img_ts = ts_map[i,1]
            if (img_ts != "nan"):
                path_img = os.path.join(path_dataset, path_folder_img, img_ts+".png")

        # DRAE_to_DZYX return real number
        # if ((osp.exists(osp.join(args.dir, path_save_dzyx_c_npy_tail, ts_map[i,0]+".npy")) 
        #     and args.shm)
        # or osp.exists(os.path.join(path_dataset,path_save_fig_tail, "frame" + str(i).zfill(4)+".png")) 
        #     and (args.sviz)):
        #     continue
        arrDZYX = DRAE_to_DZYX(path_dataset, file)
        # arrDZYX[arrDZYX < 1.] = 1.
        # log_arrDZYX = np.log2(arrDZYX)
        # delog_arrDZYX = 2**log_arrDZYX
        # print(np.min(arrDZYX), np.max(arrDZYX))
        # print(np.min(log_arrDZYX), np.max(log_arrDZYX))
        # print(np.min(delog_arrDZYX), np.max(delog_arrDZYX))

        # continue
        if (args.shm):
            os.remove(file)
            # convert to log magnitude float16
            arrDZYX[arrDZYX <= 1.] = 1. # To avoid inf value when applying log2 transform
            arrDZYX = np.log2(arrDZYX).astype(np.float16)
            if (not osp.exists(osp.join(args.dir, path_save_dzyx_c_npy_tail, ts_map[i,0]+".npy"))):
                np.save(osp.join(args.dir, path_save_dzyx_c_npy_tail, ts_map[i,0]+".npy"),
                        arrDZYX)

        if (args.sviz or args.viz):
            viz_YX_YZ_ZX_CAM(arrDZYX[0,:,:,:], os.path.join(path_dataset,path_save_fig_tail,
                    "frame" + str(i).zfill(4)+".png"), [x_min, x_max, y_min, y_max, z_min, z_max],
                    path_img, anim=args.viz)
        

        ed = time.time()
        print(ed - st)

if __name__ == "__main__":
    main()
