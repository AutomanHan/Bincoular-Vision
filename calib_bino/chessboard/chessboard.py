import cv2 
import numpy as np

width = 300
height = 400
length = 50

image = np.zeros((width,height),dtype = np.uint8)
print(image.shape[0],image.shape[1])

for j in range(height):
    for i in range(width):
        if(((int)(i/length) + (int)(j/length))%2==1):
            image[i,j] = 255;
cv2.imwrite("chess.jpg",image)
cv2.imshow("chess",image)
cv2.waitKey(0)
