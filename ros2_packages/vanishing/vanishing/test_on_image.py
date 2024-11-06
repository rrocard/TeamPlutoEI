import cv2 as cv
import sys
import my_line_library as mll

img1=cv.imread(cv.samples.findFile("flow-frame-001.jpg"))
img2=cv.imread(cv.samples.findFile("flow-frame-002.jpg"))

width=img1.shape[1]
height=img1.shape[0]

h=height//2

int1=mll.intensity_mesure(img1,h)
int2=mll.intensity_mesure(img2,h)


mll.draw_function(img1,int1,0,height,0,int1[0],(0,0,255),2) #passé
mll.draw_function(img1,int2,0,height,0,int1[0],(0,255,255),2) #futur


shiftbis=mll.local_shift(int1,int2)

diff=-shiftbis+int1[0]

mll.draw_function(img1,diff,0,height,0,int1[0],(255,255,0),1)

cv.imshow("Display",img1)
cv.waitKey(0)