import cv2 as cv
import numpy as np
from image_processing import load_image

def make_edge_image(image, lower, upper, new_name):
    img_BGR = load_image(image)
    black_white = cv.cvtColor(img_BGR, cv.COLOR_BGR2GRAY)

    # finding edges (new image array)
    edges = cv.Canny(black_white, lower, upper)
    cv.imwrite('edges_' + new_name, edges)

    # contours of edge image (list of image arrays)
    image, contours, hierarchy = cv.findContours(edges, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)
    cv.imwrite('edge-contours_' + new_name, cv.drawContours(img_BGR.copy(), contours, -1, (0, 255, 0), 1))
    cv.imwrite('just_edge-contours_' + new_name, cv.drawContours(np.zeros(img_BGR.shape), contours, -1, (0, 255, 0), 1))

    # contours of original image in black and white
    # image, contours, hierarchy = cv.findContours(black_white, cv.RETR_TREE,  cv.CHAIN_APPROX_NONE)
    # cv.imwrite('bw-contours_' + new_name, cv.drawContours(img_BGR.copy(), contours, -1, (0, 255, 0), 1))
    # cv.imwrite('just_bw-contours_' + new_name, cv.drawContours(np.zeros(img_BGR.shape), contours, -1, (0, 255, 0), 1))

if __name__ == '__main__':
    #make_edge_image('test_images/blue_mug.jpeg', 200, 300, 'mug.jpeg')
    make_edge_image('test_images/dog.jpg', 50, 300, 'dog.jpg')
