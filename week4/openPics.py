import numpy as np
import cv2
from color_segmentation import *

for i in range(1, 36):
    picture = np.load('/Users/arnavgupta/Beaverworks/week4/pictures/{}.npy'.format(i))

    drive, pic, picture, a = signIdentify(picture)
    cv2.imshow("orial", picture)

    cv2.imshow("htest", a)

    print(drive)
    cv2.imshow("original", pic)
    cv2.waitKey()

for i in range(50, 74):
    picture = np.load('/Users/arnavgupta/Beaverworks/week4/pictures/{}.npy'.format(i))

    drive, pic, picture, a = signIdentify(picture)
    cv2.imshow("orial", picture)

    cv2.imshow("htest", a)

    print(drive)
    cv2.imshow("original", pic)




    # print(_, __)


    cv2.waitKey()