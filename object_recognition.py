import cv2 as cv # openCV
import numpy as np
import image_processing as img

'''
Draws a neon green outline around a single solid-colored blue object in the image file.

original_image - input image file in, as a BGR array.
write_image - boolean (should we save the image file?)
filename - name for image if we are saving the output image file.
'''
def outline_blue(original_image, write_image, filename):
    new_image = original_image.copy()

    # find color range
    blue = np.uint8([[[255, 0, 0 ]]])
    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
    lower_blue = np.array([hsv_blue[0][0][0] - 50, 50, 50]) # in HSV
    upper_blue = np.array([hsv_blue[0][0][0] + 50, 255, 255]) # in HSV

    # find contours in color range
    image_HSV = img.convert_to_HSV(original_image)
    mask = cv.inRange(image_HSV, lower_blue, upper_blue)
    image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

    # draw all contours
    # cv.drawContours(new_image, contours, -1, (0, 255, 0), 3)

    # draw largest contour
    contours.sort(key = cv.contourArea, reverse = True)
    cv.drawContours(new_image, contours, 0, (0, 255, 0), 3)

    if write_image:
        cv.imwrite(filename, new_image)

    return new_image

'''
Draws a neon green box around a single solid-colored blue object in the image file.

original_image - input image file in, as a BGR array.
write_image - boolean (should we save the image file?)
filename - name for image if we are saving the output image file.
'''
def box_blue(original_image, write_image, filename):
    new_image = original_image.copy()

    # find color range
    blue = np.uint8([[[255, 0, 0 ]]])
    hsv_blue = cv.cvtColor(blue, cv.COLOR_BGR2HSV)
    lower_blue = np.array([hsv_blue[0][0][0] - 50, 50, 50]) # in HSV
    upper_blue = np.array([hsv_blue[0][0][0] + 50, 255, 255]) # in HSV

    # find contours in color range
    image_HSV = img.convert_to_HSV(original_image)
    mask = cv.inRange(image_HSV, lower_blue, upper_blue)
    image, contours, hierarchy = cv.findContours(mask, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)

    # find rectangle bounding largest contour in color range
    contour = max(contours, key = cv.contourArea)
    x, y, w, h = cv.boundingRect(contour)

    # draw rectangle
    cv.rectangle(new_image, (x, y), (x + w, y + h), (0, 255, 0), 3)

    if write_image:
        cv.imwrite(filename, new_image)

    return new_image

'''
Draws a neon green outline around the blue objects in the webcam feed.
'''
def outline_blue_webcam():
    camera_feed = cv.VideoCapture(0)
    cv.namedWindow('webcam feed')

    while True:
        returned_frame, frame = camera_feed.read()
        if not returned_frame:
            break

        key = cv.waitKey(1)
        if key % 256 == 27:
            break # esc key pressed

        outlined_frame = outline_blue(frame, False, None)
        cv.imshow('webcam feed', outlined_frame)

    camera_feed.release()
    cv.destroyAllWindows()

'''
Draws a neon green box around the blue objects in the webcam feed.
'''
def box_blue_webcam():
    camera_feed = cv.VideoCapture(0)
    cv.namedWindow('webcam feed')

    while True:
        returned_frame, frame = camera_feed.read()
        if not returned_frame:
            break

        key = cv.waitKey(1)
        if key % 256 == 27:
            break # esc key pressed

        boxed_frame = box_blue(frame, False, None)
        cv.imshow('webcam feed', boxed_frame)

    camera_feed.release()
    cv.destroyAllWindows()

if __name__ == '__main__':
    # outline_blue(img.load_image('blue_square.jpg'), True, 'outlined_blue_square.jpg')
    # outline_blue(img.load_image('blue_mug.jpeg'), True, 'outlined_blue_mug.jpeg')

    # box_blue(img.load_image('blue_square.jpg'), True, 'boxed_blue_square.jpg')
    # box_blue(img.load_image('blue_mug.jpeg'), True, 'boxed_blue_mug.jpeg')

    # outline_blue_webcam()
    # box_blue_webcam()
