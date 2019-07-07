import cv2
import numpy as np
H = np.array([[-1.00000000e+00,  2.23595438e-15,   5.00000000e+02],
              [6.93848929e-14, -1.00000000e+00,  5.00000000e+02],
              [1.41349332e-16,  1.15943374e-18,  1.00000000e+00]])
H = H/2
def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (640, 480))
    
    return im_out

cam1 = cv2.VideoCapture(1)
#cam2 = cv2.VideoCapture(2)
while True:
    s1, img1 = cam1.read() # captures image
    #s2, img2 = cam2.read()
    
    wrp = warp_image(img1, H)
    cv2.imshow('image1', wrp)
    #cv2.imshow('image2', img2)
    

    if cv2.waitKey(1)==27:
        break
cv2.destroyAllWindows()

