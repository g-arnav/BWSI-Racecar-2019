import cv2
#import imutils
import numpy as np
import pdb
import math


def cd_color_segmentation(img, low_range, high_range, show_image=False):
    """
	Implement the cone detection using color segmentation algorithm
	    Input:
	    img: np.3darray; the input image with a cone to be detected
	Return:
	    bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
		    (x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    """
    # convert from rgb to hsv color space (it might be BGR)
    new_img = cv2.cvtColor(img[175:], cv2.COLOR_BGR2HSV)

    # define lower and upper bound of image values
    # TO DO!

    # create mask for image with overlapping values
    mask = cv2.inRange(new_img, low_range, high_range)

    # filter the image with bitwise and
    filtered = cv2.bitwise_and(new_img, new_img, mask=mask)

    # find the contours in the image
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    vx, vy, x0, y0 = 0, 0, 0, 0
    cnt = np.count_nonzero(filtered[:])
    #print("looking for contours")
    if len(contours) != 0:
        #print("found a contour")
        # find contour with max area, which is most likely the cone
        # Solution note: max uses an anonymous function in this case, we can also use a loop...
        contours_max = max(contours, key=cv2.contourArea)
        #print(type(filtered))
        # Find vector based on contour
        vx, vy, x0, y0 = cv2.fitLine(contours_max, cv2.DIST_L2, 0, 0.01,0.01)


        # Draw the bounding rectangle
        """ print(x0)
        print(y0)
        print(vx)
        print(vy) """
        vx *=  1000
        vy*= -1000

        cv2.line(filtered, (x0,y0), (x0+vx, y0-vy), (0, 0, 255))

    if show_image:
        cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
        if key == 'q':
            cv2.destroyAllWindows()

    # Return bounding box
    if cnt > 15000:
        print(x0, y0, vx, vy)
        a = -math.pi/2 + math.atan2(vy, vx)
        return x0, filtered, cnt
    else:
        return None

# signIdentify('./images/leftTemplate.png', './images/onewayarrow.png')
