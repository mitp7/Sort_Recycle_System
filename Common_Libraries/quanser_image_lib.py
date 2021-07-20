import numpy as np
import cv2


# Convert a colour image to HSV, filter for hues within a certain width of center
# hue_center - The hue being searched for
# hue_width - The width of the range of hue values to accept
# hue_gamut - The max hue value. Open CV uses 180, set this to 255 for normal hue values
def hue_threshold(image, hue_center = 0, hue_width = 20, hue_gamut = 180):

    invert = False
    # Scale for incompatible hue gamut
    if hue_gamut != 180:
        scale = 180/hue_gamut
        hue_center = scale * hue_center
        hue_width = scale * hue_width

    # Set min and max hue values and general limits for saturation and value
    h_min = (hue_center - (hue_width/2)) % 180
    h_max = (hue_center + (hue_width/2)) % 180
    sv_min = 64
    sv_max = 255

    # Convert and threshold image
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    bottom_bounds = np.array([0.0,sv_min,sv_min])
    lower_bounds = np.array([h_min,sv_min,sv_min])
    upper_bounds = np.array([h_max,sv_max,sv_max])
    top_bounds = np.array([179.9,sv_max,sv_max])

    #Threshold for hue, special case for values that wrap past zero
    if h_min < h_max:
        binary = cv2.inRange(image_hsv, lower_bounds, upper_bounds)
    else:
        binary_low = cv2.inRange(image_hsv, bottom_bounds, upper_bounds)
        binary_high = cv2.inRange(image_hsv, lower_bounds, top_bounds)
        binary = cv2.bitwise_or(binary_low, binary_high)
	
    return binary

# Crop an image to extract a region of interest using x and y ranges of pixels
def crop_rect(image, x_range = [0,0], y_range = [0,0]):
    
    if x_range[1] > x_range[0] and y_range[1] > y_range[0]:
        image_crop = image[y_range[0]:y_range[1], x_range[0]:x_range[1]]
    else:
        image_crop = image

    return image_crop

# Draw a rectangle over an image to indicate the region of interest
def show_ROI(image, x_range = [0,0], y_range = [0,0]):

    image = cv2.rectangle(image, (x_range[0], y_range[0]), (x_range[1], y_range[1]), (255, 128, 128), 2)

    return image

# Draw the rectangle ROI with a vertical line indicating the target center
def show_ROI_target(image, x_range = [0,0], y_range = [0,0], targ = -1):
    
    image = show_ROI(image, x_range, y_range)
    
    t_range = [y_range[0] - 10, y_range[1] + 10]

    if targ == -1:
        image = cv2.line(image, (320, t_range[0]), (320, t_range[1]), (0, 0, 255), 4)
    else:
        t_x = int(round(targ))
        image = cv2.line(image, (t_x, t_range[0]), (t_x, t_range[1]), (128, 255, 128), 4)

    return image

# Find the center of a line in a thresholded image
def extract_line_ctr(image):

    center = -1

    # Average the pixels in each column and find the maximum value
    column_vals = np.mean(image, axis = 0)
    max_col = np.amax(column_vals)

    #Check for lost line and return the average x position of max values. Else, return -1
    if max_col > 64:
        center = np.mean(np.argwhere(column_vals == max_col))

    return center
