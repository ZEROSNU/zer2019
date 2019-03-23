import cv2
import numpy as np
import time

#source point setting
# 1  4
# 2  3

#mode
# 0:left cam  /  1:front cam  /  2:right cam
mode = 0
cam_set = 0
src_points = [0, 0, 0, 0]

cam_str = "front"

def setPoints(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        global cam_set, src_points, src
        if cam_set < 4:
            src_points[cam_set] = np.array([x,y])
            print(cam_str + " camera point " + str(cam_set) + " set x : " + str(x) + " y : " + str(y))
            cam_set = cam_set + 1
            if cam_set == 4:
                src = np.vstack((src_points[0],src_points[1], src_points[2],src_points[3]))
                print(cam_str + " camera setting done")
                return True
            
            return True
        else:
            return True




#get source points

cam = cv2.VideoCapture(1)

while True:
    s_cam, img_cam = cam.read()
    cv2.imshow('cam', img_cam)

    cv2.setMouseCallback('cam', setPoints)

    if cv2.waitKey(1) & 0xFF == 27:
        break
cv2.destroyAllWindows()


#dest points
'''
#default
dest_1 = np.array([300,300]) # left front   dest1   dest4
dest_2 = np.array([300,600]) # left rear    dest2   dest3
dest_3 = np.array([600,600]) # right rear
dest_4 = np.array([600,300]) # right front
'''

#for front cam

dest_1 = np.array([365,300-60]) #
dest_2 = np.array([185,300-60]) # 
dest_3 = np.array([185,300+60]) #
dest_4 = np.array([365,300+60]) #

#for left cam
'''
dest_1 = np.array([365,300-120]) #
dest_2 = np.array([185,300-120]) # 
dest_3 = np.array([185,300-60]) #
dest_4 = np.array([365,300-60]) #
'''


dest = np.vstack((dest_1, dest_2, dest_3, dest_4))

rows = []
for i in range(0,4):
    r1 = np.append(-src[i], [-1,0,0,0, src[i][0]*dest[i][0], src[i][1]*dest[i][0],dest[i][0]])
    r2 = np.append([0,0,0], [-src[i][0],-src[i][1],-1, src[i][0]*dest[i][1], src[i][1]*dest[i][1],dest[i][1]])
    rows.append(r1)
    rows.append(r2)

A = np.vstack(rows)
svd = np.linalg.svd(A)
V = svd[2]

h = V[-1]/V[-1][-1]

#Final Homography Matrix
H = np.vstack((h[0:3],h[3:6],h[6:9]))


def warp_image(image, homography):
    im_out = cv2.warpPerspective(image, homography, (600, 600))
    
    return im_out

im_src = img_cam
img_warp = warp_image(im_src, H)

cv2.imshow('image',img_warp)
cv2.waitKey(0)
cv2.destroyAllWindows()

print H