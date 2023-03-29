# Copyright (c) 2023 Ali Syaugi Bilfagih

import cv2
import os
def variance_of_laplacian(image):
	# compute the Laplacian of the image and then return the focus
	# measure, which is simply the variance of the Laplacian
	return cv2.Laplacian(image, cv2.CV_64F).var()

# Directory of picture files
path='/home/pi/takenPictures'
# The program will run in loop until it processed all pictures in the folder.  
x=1
while True : 
    # defining image as object
    image=cv2.imread('/home/pi/takenPictures/'+str(x)+'.jpg')
    # grayscaling the picture
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # laplacian kernel of the grayscaled picture
    fm = variance_of_laplacian(gray)
    text = "Not Blurry"
    # comparing the grayscaled picture laplacian kernel return value with the threshold
    if fm < 1000:
        text = "Blurry"
        cv2.putText(image, "{}: {:.2f}".format(text, fm), (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
        # save the picture with caption "Blurry" in "Blurred" folder
        cv2.imwrite('/home/pi/takenPictures/Blurred/'+str(x)+'.jpg',image)
    x += 1
    if x>6 :
        break
    #cv2.waitKey(0)
