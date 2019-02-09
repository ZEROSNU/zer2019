import cv2

cam1 = cv2.VideoCapture(1)
#cam2 = cv2.VideoCapture(2)
while True:
    s1, img1 = cam1.read() # captures image
    #s2, img2 = cam2.read()
    cv2.imshow('image1', img1)
    #cv2.imshow('image2', img2)
    

    if cv2.waitKey(1)==27:
        break
cv2.destroyAllWindows()