import cv2
import numpy as np

#src points
src_1 = np.array([0,0])
src_2 = np.array([500,0])
src_3 = np.array([500,500])
src_4 = np.array([0,500])
src = np.vstack((src_1,src_2, src_3,src_4))

#dest points
dest_1 = np.array([500,500])
dest_2 = np.array([0,500])
dest_3 = np.array([0,0])
dest_4 = np.array([500,0])
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
    im_out = cv2.warpPerspective(image, homography, (500, 500))
    
    return im_out

im_src = cv2.imread('./1.png')
img_warp = warp_image(im_src, H)

cv2.imshow('image',img_warp)
cv2.waitKey(0)
cv2.destroyAllWindows()

print H