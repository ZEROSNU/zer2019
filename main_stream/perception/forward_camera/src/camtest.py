import cv2

cam = cv2.VideoCapture(1)

while True:
    _, img = cam.read()
    cv2.imshow('img',img)

    key = cv2.waitKey(1)
    if key == ord('q'):
        break