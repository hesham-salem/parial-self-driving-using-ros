import cv2
import numpy as np


def to_hls(img):
    """
    Convert a RGB image to HLS
    :param img: RGB image
    :return: an HLS image
    """
    return cv2.cvtColor(img, cv2.COLOR_RGB2HLS)


def to_lab(img):
    """
    Convert a RGB image to LAB
    :param img:  RGB image
    :return: a LAB image
    """
    return cv2.cvtColor(img, cv2.COLOR_RGB2LAB)


def filter(img, min, max):
    """
    Takes an image and convert it to binary image
    based on the (min, max) thresholds
    :param img: the source image
    :param min: the min threshold
    :param max: the max threshold
    :return: the binary image based on the given thresholds
    """
    binary = np.zeros_like(img)
    binary[(img >= min) & (img <= max)] = 1
    return binary


def mask_with_polygon(img, vertices):
    """
    Takes an image and a set of vertices that represents a polygon
    and mask the original image with the polygon in a way that only
    the polygon is shown on the output
    :param img: the source image
    :param vertices: array of points that represents a polygon
    :return: the image with the mask applied
    """

    mask = np.zeros_like(img)

    if len(img.shape) > 2:
        channel_count = img.shape[2]
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    # filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, [vertices], ignore_mask_color)

    # returning the image only where mask pixels are non-zero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def sliding_window(binary_warped, nwindows=10):
    """
    Using sliding window to determine the left/right lane points
    :param binary_warped: the source image (should be after binary filtering and perspective wrap)
    :param nwindows: the number of windows
    :return: right/left 2nd order polynoms that matchs the lane's shape
    """
    # Take a histogram of the bottom half of the image
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)

    # Find the peak of the left and right halves of the histogram
    # These will be the starting point for the left and right lines
    midpoint = np.int(histogram.shape[0] // 2)
    quarter_point = np.int(midpoint // 2)

    # Previously the left/right base was the max of the left/right half of the histogram
    # this changes it so that only a quarter of the histogram (directly to the left/right) is considered
    leftx_base = np.argmax(histogram[quarter_point:midpoint]) + quarter_point
    rightx_base = np.argmax(histogram[midpoint:(midpoint + quarter_point)]) + midpoint

    window_height = np.int(binary_warped.shape[0] / nwindows)

    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base

    # Set the width of the windows +/- margin
    margin = 80

    # Set minimum number of pixels found to recenter window
    min_pixels_recenter = 40

    # Create empty lists to receive left and right lane pixel indices
    left_lane_idx = []
    right_lane_idx = []
    rectangle_data = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        rectangle_data.append((win_y_low, win_y_high, win_xleft_low, win_xleft_high, win_xright_low, win_xright_high))

        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
            nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
            nonzerox < win_xright_high)).nonzero()[0]

        # Append these indices to the lists
        left_lane_idx.append(good_left_inds)
        right_lane_idx.append(good_right_inds)

        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > min_pixels_recenter:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > min_pixels_recenter:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))

    # Concatenate the arrays of indices
    left_lane_idx = np.concatenate(left_lane_idx)
    right_lane_idx = np.concatenate(right_lane_idx)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_idx]
    lefty = nonzeroy[left_lane_idx]
    rightx = nonzerox[right_lane_idx]
    righty = nonzeroy[right_lane_idx]

    left_fit, right_fit = (None, None)

    # Fit a second order polynomial to each
    if len(leftx) != 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    if len(rightx) != 0:
        right_fit = np.polyfit(righty, rightx, 2)

    visualization_data = (rectangle_data, histogram)

    return left_fit, right_fit, left_lane_idx, right_lane_idx, visualization_data


def lane_polyfit(binary_warped, left_fit_prev, right_fit_prev):
    """
    Approximate based on previous fits the new lane polynomial
    :param binary_warped: the image after binary filtering and perspective wrap
    :param left_fit_prev: the previous left fit
    :param right_fit_prev: the previous right fit
    :return: the new polynomials for the lanes
    """
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    margin = 80

    left_lane_inds = (
        (nonzerox > (left_fit_prev[0] * (nonzeroy ** 2) + left_fit_prev[1] * nonzeroy + left_fit_prev[2] - margin)) &
        (nonzerox < (left_fit_prev[0] * (nonzeroy ** 2) + left_fit_prev[1] * nonzeroy + left_fit_prev[2] + margin)))

    right_lane_inds = (
        (nonzerox > (right_fit_prev[0] * (nonzeroy ** 2) + right_fit_prev[1] * nonzeroy + right_fit_prev[2] - margin)) &
        (nonzerox < (right_fit_prev[0] * (nonzeroy ** 2) + right_fit_prev[1] * nonzeroy + right_fit_prev[2] + margin)))

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    left_fit_new, right_fit_new = (None, None)
    if len(leftx) != 0:
        # Fit a second order polynomial to each
        left_fit_new = np.polyfit(lefty, leftx, 2)
    if len(rightx) != 0:
        right_fit_new = np.polyfit(righty, rightx, 2)
    return left_fit_new, right_fit_new, left_lane_inds, right_lane_inds


def lane_departure(bin_img, l_fit, r_fit, lane_width_meters=3.7):
    """
    Calculate the deprature from the center of the lane, based on the idea that
    the camera sits on the middle of the windshield
    :param bin_img: the source image
    :param l_fit: the left lane fit
    :param r_fit: the right lane fit
    :param lane_width_meters: the lane approx width in meters (defaults to 3.7 m)
    :return: the distance from the center, values > 0 represents right departure, values < 0 represents left departure
    """
    xm_per_pix = lane_width_meters / 378

    center_dist = 0
    h = bin_img.shape[0]

    # Distance from center is image x midpoint - mean of l_fit and r_fit intercepts
    if r_fit is not None and l_fit is not None:
        car_position = bin_img.shape[1] / 2
        l_fit_x_int = l_fit[0] * h ** 2 + l_fit[1] * h + l_fit[2]
        r_fit_x_int = r_fit[0] * h ** 2 + r_fit[1] * h + r_fit[2]
        lane_center_position = (r_fit_x_int + l_fit_x_int) / 2
        center_dist = (car_position - lane_center_position) * xm_per_pix

    return center_dist
