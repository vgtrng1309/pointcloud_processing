# """
# Author: trung.l.nguyen@tuni.fi
# Date  : 11-03-2025
# Desc  : This script is for converting 4D Radar DRAE tensor into 4D Radar DZYX tensor
#         for training and testing data with RT-POSE model. Inspired from data processing
#         script of RT-POSE.
# """
cimport cython
import numpy as np
cimport numpy as cnp
import math
import re
import os
import argparse
import itertools
import time

path_save_dzyx_c_npy_tail =  'radar/npy_DZYX_real/'
path_save_fig_tail = 'radar/figure/'
path_folder_img = 'images/'
path_mapping_file = 'calib/radarpcl2images_mapping.txt'
path_folder_tail = 'radar/mat/'

# @cython.boundscheck(False)
# @cython.wraparound(False)
# find index for blinear interpolation
cdef float DEG2RAD = 3.14159265 / 180.
cdef float RAD2DEG = 180. / 3.14159265
cdef int[4] DRAE_SHAPE = [16, 256, 128, 32]

cdef float x_min       = 0     #(m)
cdef float x_per_bin   = 15.1836 / 256     #(m) 15.1836 / 256
cdef float x_max       = 15.1836  #(m) x_max = range size +x_per_bin; 15.1836

cdef float y_min       = -14.953     #(m) # y_max = np.sin(max_azimuth)*max_range = np.sin(80)*15.1836
cdef float y_per_bin   = 14.953*2/128     #(m) #     = 14.953
cdef float y_max       = 14.953   #(m)   => y_res = 14.953*2 / 128 = 0.233640625

cdef float z_min       = -5.193    #(m)      z_max = np.sin(max_elevation)*max_range = np.sin(20)*15.1836
cdef float z_per_bin   = 5.193*2/32  #(m)           = 5.193
cdef float z_max       = 5.193   #(m)     => z_res = 5.193*2 / 32 = 0.3245625

cdef cnp.ndarray arr_x, arr_y, arr_z 
arr_x = np.arange(x_min,x_max,x_per_bin)
arr_y = np.arange(y_min,y_max,y_per_bin)
arr_y[abs(arr_y) < 1e-10] = 0.0
# print(arr_y.shape)
arr_z = np.arange(z_min,z_max,z_per_bin)

cdef int len_x = len(arr_x)
cdef int len_y = len(arr_y)
cdef int len_z = len(arr_z)
cdef int len_d = 16 # 16

cdef cnp.ndarray arrAzimuth, arrElevation, arrRange, arrAzimuthRad, arrElevationRad
arrAzimuth      = np.arange(0,160,160/128)
arrElevation    = np.arange(0,40,40/32)
arrRange        = np.arange(0,15.1836,15.1836/256)

arrAzimuthRad   = arrAzimuth*DEG2RAD
arrElevationRad = arrElevation*DEG2RAD

#set min/max r,a,e 
cdef float r_max, r_min, r_res, a_max, a_min, a_res, e_max, e_min, e_res
r_max = 15.1836
r_min = 0.
r_res = r_max / 256
a_max = 160.*DEG2RAD
a_min = 0.*DEG2RAD
a_res = 160./128*DEG2RAD
e_max = 40.*DEG2RAD
e_min = 0.*DEG2RAD
e_res = 40./32*DEG2RAD

# scan all xyz <=> rae
st = time.time()
cdef cnp.ndarray XYZ_combs, xyz_combs, rea_combs, tmp
XYZ_combs = np.array(list(itertools.product(*[np.arange(0,256,1), 
                                                np.arange(0,128,1),
                                                np.arange(0,32,1)])))
xyz_combs = np.array(list(itertools.product(*[arr_x, arr_y, arr_z])))

rea_combs = np.zeros((xyz_combs.shape[0], xyz_combs.shape[1]), dtype=np.float)
# r
rea_combs[:,0] = np.sqrt(xyz_combs[:,0]**2 + xyz_combs[:,1]**2 + xyz_combs[:,2]**2)
# a
tmp = xyz_combs[:,1]==0
rea_combs[tmp, 2] = np.pi / 2
rea_combs[~tmp, 2] = np.arctan(xyz_combs[~tmp,0]/xyz_combs[~tmp,1])
rea_combs[rea_combs[:,2]<0, 2] += np.pi
rea_combs[:,2] = 170*DEG2RAD-rea_combs[:,2]

# e
tmp = (xyz_combs[:,0]**2 + xyz_combs[:,1]**2) == 0
rea_combs[tmp, 1] = np.pi / 2
rea_combs[~tmp, 1] = np.arctan(xyz_combs[~tmp,2] / np.sqrt(xyz_combs[~tmp,0]**2 + xyz_combs[~tmp,1]**2))
rea_combs[:,1] += 20*DEG2RAD

cdef cnp.ndarray valid_ind, valid_ind_a, valid_ind_e, valid_ind_r
valid_ind_r = np.logical_and(rea_combs[:,0] >= r_min, rea_combs[:,0] <= r_max)#-r_res)
valid_ind_e = np.logical_and(rea_combs[:,1] >= e_min, rea_combs[:,1] <= e_max)#-e_res)
valid_ind_a = np.logical_and(rea_combs[:,2] >= a_min, rea_combs[:,2] <= a_max)#-a_res)
valid_ind   = np.logical_and(np.logical_and(valid_ind_r, valid_ind_e), valid_ind_a)

# rae => RAE
# print(xyz_combs[2000:2030,:])
# print(rea_combs[2000:2030,0])
cdef cnp.ndarray r_bin_index, e_bin_index, a_bin_index
r_bin_index = (rea_combs[valid_ind,0] // r_res).astype(np.int32)
e_bin_index = (rea_combs[valid_ind,1] // e_res).astype(np.int32) # Adjust back to bin index
a_bin_index = (rea_combs[valid_ind,2] // a_res).astype(np.int32) # Adjust back to bin index

cdef cnp.ndarray r_bin_next_index, e_bin_next_index, a_bin_next_index
r_bin_next_index = r_bin_index + 1
r_bin_next_index[r_bin_next_index > 255] = 255
e_bin_next_index = e_bin_index + 1
e_bin_next_index[e_bin_next_index > 31] = 31
a_bin_next_index = a_bin_index + 1
a_bin_next_index[a_bin_next_index > 127] = 127

cdef cnp.ndarray r_index_prev, r_index_next, e_index_prev, \
                 e_index_next, a_index_prev, a_index_next

r_index_prev = rea_combs[valid_ind,0] - arrRange[r_bin_index]
r_index_next = arrRange[r_bin_next_index] - rea_combs[valid_ind,0]
e_index_prev = rea_combs[valid_ind,1] - arrElevationRad[e_bin_index]
e_index_next = arrElevationRad[e_bin_next_index] - rea_combs[valid_ind,1]
a_index_prev = rea_combs[valid_ind,2] - arrAzimuthRad[a_bin_index]
a_index_next = arrAzimuthRad[a_bin_next_index] - rea_combs[valid_ind,2]

# Valid XYZ
XYZ_combs = XYZ_combs[valid_ind,:]

en = time.time()
print(en-st)

@cython.boundscheck(False)
@cython.wraparound(False)
def DRAE_to_DZYX(path_dataset, file):
    cdef cnp.ndarray arr4DRadar, arr4DRadar_imag, arr4DRadar_real
    cdef cnp.ndarray arrDZYX
    cdef cnp.ndarray rea000, rea001, rea010, rea011, rea100, rea101, rea110, rea111 
    cdef float del_cross = 1/(r_res*a_res*e_res)
    
    s_file_seq = str(file)
    path = os.path.join(path_dataset,path_folder_tail, s_file_seq)
    path_save_dzyx_c_npy = os.path.join(path_dataset,path_save_dzyx_c_npy_tail, s_file_seq)
    # print('Save path: ',path_save_dzyx_c_npy)
    
    # TODO: Enable this
    #-----loading data-----
    ## name 'matr4' is setting from matlab
    arr4DRadar = np.fromfile(os.path.join(path), dtype=np.float32)
    arr4DRadar_real = arr4DRadar[:16*256*128*32]
    arr4DRadar_imag = arr4DRadar[16*256*128*32:]
    arr4DRadar = arr4DRadar_real + 1j*arr4DRadar_imag
    arr4DRadar = arr4DRadar.reshape(DRAE_SHAPE, order="F")
    
    arr4DRadar = np.abs(arr4DRadar)

    # arr4DRadar = np.load(os.path.join(path)) # DRAE
    # print(arr4DRadar)
    #-----loading data-----end
    arr4DRadar = np.transpose(arr4DRadar,(0,1,3,2)) # DREA
    # Creat XYZ  numpy
    # arrZYX = np.ones((len_z,len_y,len_x), dtype='complex64')
    # arrDZYX = np.zeros((len_d,len_z,len_y,len_x), dtype='complex64')
    arrDZYX = np.zeros((len_d,len_z,len_y,len_x), dtype=np.float32)

    # print(x_bin_index[valid_ind])

    # Assigning value
    rea000 = arr4DRadar[:, r_bin_index, e_bin_index, a_bin_index]
    rea001 = arr4DRadar[:, r_bin_index, e_bin_index, a_bin_next_index]
    rea010 = arr4DRadar[:, r_bin_index, e_bin_next_index, a_bin_index]
    rea011 = arr4DRadar[:, r_bin_index, e_bin_next_index, a_bin_next_index]
    rea100 = arr4DRadar[:, r_bin_next_index, e_bin_index, a_bin_index]
    rea101 = arr4DRadar[:, r_bin_next_index, e_bin_index, a_bin_next_index]
    rea110 = arr4DRadar[:, r_bin_next_index, e_bin_next_index, a_bin_index]
    rea111 = arr4DRadar[:, r_bin_next_index, e_bin_next_index, a_bin_next_index]

    # print(rea000.shape,rea001.shape,rea010.shape,rea011.shape,rea100.shape,rea101.shape,
    #       rea110.shape,rea111.shape)
    # print(arrDZYX[:,XYZ_combs[:,2], XYZ_combs[:,1], XYZ_combs[:,0]].shape)

    arrDZYX[:,XYZ_combs[:,2], XYZ_combs[:,1], XYZ_combs[:,0]] = \
        del_cross*(rea000*(r_index_next)*(e_index_next)*(a_index_next)
                    + rea001*(r_index_next)*(e_index_next)*(a_index_prev)
                    + rea010*(r_index_next)*(e_index_prev)*(a_index_next)
                    + rea011*(r_index_next)*(e_index_prev)*(a_index_prev)
                    + rea100*(r_index_prev)*(e_index_next)*(a_index_next)
                    + rea101*(r_index_prev)*(e_index_next)*(a_index_prev)
                    + rea110*(r_index_prev)*(e_index_prev)*(a_index_next)
                    + rea111*(r_index_prev)*(e_index_prev)*(a_index_prev))
    #doing reverse of XYZ axis =Y
    arrDZYX = np.flip(arrDZYX,axis=2)

    # np.save(os.path.join(path_save_dzyx_c_npy),arrDZYX)
    # Save fig
    # print(np.max(arr4DRadar), np.min(arr4DRadar))
    return arrDZYX

