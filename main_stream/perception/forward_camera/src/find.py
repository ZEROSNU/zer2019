import cv2

cam1 = cv2.VideoCapture(1)
while True:
    s1, img1 = cam1.read() # captures image
    cv2.imshow('image1', img1)
 
    

    if cv2.waitKey(1)==27:
        break
cv2.destroyAllWindows()
