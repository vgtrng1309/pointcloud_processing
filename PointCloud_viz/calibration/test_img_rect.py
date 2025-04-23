import cv2
import numpy as np

img_left = cv2.imread("../data/test_bag_05/images/1735305229771343498.png")
img_right = cv2.imread("../data/test_bag_05/rimages/1735305229771343498.png")

img = np.hstack((img_left, img_right))
print(img.shape)

for i in np.linspace(100, 500, 5):
    # print(i)
    cv2.line(img, (0, int(i)), (img.shape[1], int(i)), (0,0,0), 3)

cv2.imshow("Test", img)
cv2.waitKey(0)
cv2.destroyAllWindows()