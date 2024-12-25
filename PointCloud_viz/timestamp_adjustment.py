import os
import shutil
import numpy as np

offset=np.float(246248409585953)

list_file = sorted(os.listdir("images"))

# Copy the content of
# source to destination
for file in list_file:
    ts = np.float(file[:-4])
    ts += offset
    nfile = "{:10.0f}".format(ts)
    shutil.copyfile("images/"+file, "images_/"+nfile)


