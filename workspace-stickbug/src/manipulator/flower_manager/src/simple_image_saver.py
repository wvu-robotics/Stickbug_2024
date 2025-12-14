import cv2

img = cv2.imread('flower_pose_plot.png')
print(img.shape, img.dtype, type(img))
print(cv2.imwrite('here.png', img))
