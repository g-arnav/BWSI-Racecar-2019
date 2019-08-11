import numpy as np
import cv2
from color_segmentation import cd_color_segmentation

picture = np.load('/Users/arnavgupta/Beaverworks/wall_follower/pictures/redyellowblue.npy')


_, pic, cnt = cd_color_segmentation(picture, np.array([100, 50, 100]), np.array([150, 255, 255]))
cv2.imshow("original", picture)
cv2.imshow("htest", pic)

print(cnt)

# print(_, __)


cv2.waitKey()