import cv2 as cv # openCV
import numpy as np

'''
Loads image as BGR numpy array.

filename - name of image to load, as string
'''
def load_image(filename):
    image_BGR = np.asarray(cv.imread(filename))
    return image_BGR

'''
HSV:
Hue range is [0,179]
Saturation range is [0,255]
Value range is [0,255]

Saves given BGR image as a new .png image file, converted to HSV.
'''
def convert_to_HSV(filename):
    image_data = load_image(filename)
    image_HSV = cv.cvtColor(image_data, cv.COLOR_BGR2HSV)
    cv.imwrite('HSV' + filename, image_HSV)
    return image_HSV


if __name__ == '__main__':
    #convert_to_HSV('test.png')
    #convert_to_HSV('dog.jpg')
    pass
