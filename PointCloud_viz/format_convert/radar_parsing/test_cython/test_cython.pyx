import numpy as np
import time
cimport numpy as cnp
cnp.import_array()

st=time.time()

cdef int offset, i, j, k, l
cdef cnp.ndarray arr
offset = 1

arr = np.random.rand(16,256,128,32)
offset = 1
for i in range(arr.shape[0]):
    for j in range(arr.shape[1]):
        for k in range(arr.shape[2]):
            for l in range(arr.shape[3]):
                arr[i,j,k,l] += offset

ed=time.time()
print(ed-st)