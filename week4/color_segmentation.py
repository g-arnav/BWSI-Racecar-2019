import cv2
import imutils
import numpy as np
import pdb


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
    new_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # define lower and upper bound of image values
    # TO DO!

    # create mask for image with overlapping values
    mask = cv2.inRange(new_img, low_range, high_range)

    # filter the image with bitwise and
    filtered = cv2.bitwise_and(new_img, new_img, mask=mask)

    # find the contours in the image
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    x1, y1, x2, y2 = 0, 0, 0, 0

    a = 0
    if len(contours) != 0:
        # find contour with max area, which is most likely the cone
        # Solution note: max uses an anonymous function in this case, we can also use a loop...
        contours_max = max(contours, key=cv2.contourArea)
        a = (cv2.contourArea(contours_max))

        # Find bounding box coordinates
        x1, y1, x2, y2 = cv2.boundingRect(contours_max)

        # Draw the bounding rectangle


    if show_image:
        cv2.imshow("Color segmentation", img)
        key = cv2.waitKey()
        if key == 'q':
            cv2.destroyAllWindows()

    # Return bounding box
    return ((x1, y1), (x1 + x2, y1 + y2)), filtered, a, contours


def startLights(img):

    im = img[:175]

    _, filtered, a, __ = cd_color_segmentation(im, np.array([0, 150, 100]), np.array([50, 255, 255]))
    print('a {}'.format(a))
    if a > 4500:
        drive = True
    else:
        drive = False

    return drive, filtered, _, a


def signIdentify(img):
    # Python program to illustrate
    # multiscaling in template matching

    # Read the main image
    # [100:200, 200:472]
    im = img[80:]
    _, img_rgb, __, contours = cd_color_segmentation(im, np.array([0, 150, 100]), np.array([30, 255, 255]))
    cntsSorted = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)

    contours_max = max(contours, key=cv2.contourArea,)
    x1, y1, x2, y2 = cv2.boundingRect(cntsSorted[0])


    contours_2max = max(contours, key=cv2.contourArea)
    x3, y3, x4, y4 = cv2.boundingRect(cntsSorted[1])

    # if x1 < x3:
    #     sign = img_rgb[y1:y1 + y2, x1:x3 + x4]
    #     cv2.rectangle(img_rgb, (x1, y1), (x3, y3), (0, 255, 0))
    # else:
    #     sign = img_rgb[y1:y1 + y2, x1:x1 + x2]
    #     cv2.rectangle(img_rgb, (x1, y1), (int((x1 + x2 + x3 + x4) / 2) + 20, int((y1 + y2 + y3 + y4) /2)), (0, 255, 0))

    if x1 > x3:
        sign = im[y3 + y4 :  y3 + y4 + 30, x3:x1+ x2]
    else:
        sign = im[y3 + y4 :  y3 + y4 + 30, x1:x3 + x4]

    sign = cv2.cvtColor(sign, cv2.COLOR_BGR2GRAY)

    _, sign = cv2.threshold(sign, 100, 255, cv2.THRESH_BINARY)


    h, w= sign.shape

    sign_left = sign[:, 0:int(w/2)]

    sign_right = sign[:, int(w/2):w]

    if np.count_nonzero(sign_left) > np.count_nonzero(sign_right):
        return 'left'#, im, sign, img_rgb
    else:
        return 'right'#, im, sign, img_rgb
