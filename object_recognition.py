import cv2 as cv # openCV
import numpy as np
import image_processing as img

'''
Outlines the blue object in the image file with neon green.

filename - name of input image file, must contain 1 solid-colored blue object.
'''
def outline_blue(filename):
    original_image = img.load_image(filename)
    new_image = original_image.copy()

    # find color range
    blue = np.uint8([[[255, 0, 0 ]]])
    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
    lower_blue = np.array([hsv_blue[0][0][0] - 50, 50, 50]) # in HSV
    upper_blue = np.array([hsv_blue[0][0][0] + 50, 255, 255]) # in HSV

    # find contours in color range
    image_HSV = img.convert_to_HSV(filename)
    mask = cv.inRange(image_HSV, lower_blue, upper_blue)
    image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

    # draw all contours
    #cv.drawContours(new_image, contours, -1, (0, 255, 0), 3)

    # draw largest contour
    contours.sort(key = cv.contourArea, reverse = True)
    cv.drawContours(new_image, contours, 0, (0, 255, 0), 3)

    cv.imwrite('outlined_' + filename, new_image)

'''
Draws a neon green box around the blue object in the image file.

filename - name of input image file, must contain 1 solid-colored blue object.
'''
def box_blue(filename):
    original_image = img.load_image(filename)
    new_image = original_image.copy()

    # find color range
    blue = np.uint8([[[255, 0, 0 ]]])
    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
    lower_blue = np.array([hsv_blue[0][0][0] - 50, 50, 50]) # in HSV
    upper_blue = np.array([hsv_blue[0][0][0] + 50, 255, 255]) # in HSV

    # find contours in color range
    image_HSV = img.convert_to_HSV(filename)
    mask = cv.inRange(image_HSV, lower_blue, upper_blue)
    image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

    # find rectangle bounding largest contour in color range
    contour = max(contours, key = cv.contourArea)
    x, y, w, h = cv.boundingRect(contour)

    # draw rectangle
    cv.rectangle(new_image, (x, y), (x + w, y + h), (0, 255, 0), 3)
    
    cv.imwrite('boxed_' + filename, new_image)


if __name__ == '__main__':
    #outline_blue('blue_square.jpg')
    #outline_blue('blue_mug.jpeg')
    #box_blue('blue_square.jpg')
    #box_blue('blue_mug.jpeg')
    pass
