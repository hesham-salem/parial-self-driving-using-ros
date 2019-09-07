from processors import *


class LaneDepartureDetector:
    """
    A class that will detect lane departure from a set of images (frames of video)
    """

    def __init__(self, perspective_src, perspective_dst, roi_matrix):
        """
        Init the class
        :param perspective_src: the source perspective wrap points (should match a trapezoid that matches the lane shape
        :param perspective_dst: the destination perspective wrap points
        :param roi_matrix: the region of interest mask
        """
        self.left_lane = Line()
        self.right_lane = Line()
        self.M = cv2.getPerspectiveTransform(perspective_src, perspective_dst)
        self.Minv = cv2.getPerspectiveTransform(perspective_dst, perspective_src)
        self.roi_matrix = roi_matrix

    def process_image(self, image):
        """
        Process a single frame of the image
        :param image: the source image
        :return: the image with discovered lanes + departure
        """
        h, w = image.shape[:2]

        # Apply some smoothing:
        blurred = cv2.blur(image, (15, 15))

        # Create the binary image of the lanes based on the color space:
        hls = to_hls(image)
        lab = to_lab(image)
        s_bin = filter(hls[:, :, 2], 220, 255)
        b_bin = filter(lab[:, :, 2], 190, 255)
        r_bin = filter(image[:, :, 2], 190, 255)
        binary = np.zeros_like(blurred[:, :, 0])
        binary[(s_bin == 1) | (b_bin == 1) | (r_bin == 1)] = 1

        # Find the Region of Interest:
        roi = mask_with_polygon(binary, self.roi_matrix)

        # Wrap the perspective into birds-eye view
        wrapped = cv2.warpPerspective(roi, self.M, (w, h), flags=cv2.INTER_LINEAR)

        (processed_image, departure) = self._find_lanes(image, wrapped)
        return processed_image, departure

    @staticmethod
    def _draw_data(original_img, center_dist):
        new_img = np.copy(original_img)

        font = cv2.FONT_HERSHEY_DUPLEX
        direction = ''

        if center_dist > 0:
            direction = 'right'
        elif center_dist < 0:
            direction = 'left'

        abs_center_dist = abs(center_dist)
        text = '{:04.3f}'.format(abs_center_dist) + 'm ' + direction + ' of center'
        cv2.putText(new_img, text, (40, 70), font, 1.5, (255, 255, 255), 2, cv2.LINE_AA)

        return new_img

    def _find_lanes(self, orig_img, img):
        new_img = np.copy(orig_img)
        d_center = 0

        if not self.left_lane.detected or not self.right_lane.detected:
            l_fit, r_fit, l_lane_inds, r_lane_inds, _ = sliding_window(img)
        else:
            l_fit, r_fit, l_lane_inds, r_lane_inds = lane_polyfit(img, self.left_lane.best_fit,
                                                                  self.right_lane.best_fit)

        # invalidate both fits if the difference in their x-intercepts isn't around 350 px (+/- 100 px)
        if l_fit is not None and r_fit is not None:
            # calculate x-intercept (bottom of image, x=image_height) for fits
            h = img.shape[0]
            l_fit_x_int = l_fit[0] * h ** 2 + l_fit[1] * h + l_fit[2]
            r_fit_x_int = r_fit[0] * h ** 2 + r_fit[1] * h + r_fit[2]
            x_int_diff = abs(r_fit_x_int - l_fit_x_int)
            if abs(350 - x_int_diff) > 120:
                l_fit = None
                r_fit = None

        self.left_lane.add_fit(l_fit, l_lane_inds)
        self.right_lane.add_fit(r_fit, r_lane_inds)

        # draw the current best fit if it exists
        if self.left_lane.best_fit is not None and self.right_lane.best_fit is not None:
            img_out1 = self._draw_lane(new_img, img, self.left_lane.best_fit, self.right_lane.best_fit)
            d_center = lane_departure(img, self.left_lane.best_fit, self.right_lane.best_fit)
            img_out = self._draw_data(img_out1, d_center)
        else:
            img_out = new_img

        return img_out, d_center

    def _draw_lane(self, original_img, binary_img, l_fit, r_fit):
        new_img = np.copy(original_img)
        if l_fit is None or r_fit is None:
            return original_img

        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_img).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        h, w = binary_img.shape
        ploty = np.linspace(0, h - 1, num=h)  # to cover same y-range as image
        left_fitx = l_fit[0] * ploty ** 2 + l_fit[1] * ploty + l_fit[2]
        right_fitx = r_fit[0] * ploty ** 2 + r_fit[1] * ploty + r_fit[2]

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (200, 200, 200))
        cv2.polylines(color_warp, np.int32([pts_left]), isClosed=False, color=(0, 0, 255), thickness=15)
        cv2.polylines(color_warp, np.int32([pts_right]), isClosed=False, color=(0, 0, 255), thickness=15)

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, self.Minv, (w, h))

        # Combine the result with the original image
        result = cv2.addWeighted(new_img, 1, newwarp, 0.5, 0)
        return result


class Line:
    def __init__(self):
        self.detected = False  # was the line detected in the last iteration?
        self.recent_xfitted = []  # x values of the last n fits of the line
        self.bestx = None  # average x values of the fitted line over the last n iterations
        self.best_fit = None  # polynomial coefficients averaged over the last n iterations
        self.current_fit = []  # polynomial coefficients for the most recent fit
        self.radius_of_curvature = None  # radius of curvature of the line in some units
        self.line_base_pos = None  # distance in meters of vehicle center from the line
        self.diffs = np.array([0, 0, 0], dtype='float')  # difference in fit coefficients between last and new fits
        self.px_count = None  # number of detected pixels

    def add_fit(self, fit, inds):
        if fit is not None:  # add the found fit to the line, up to n
            if self.best_fit is not None:
                self.diffs = abs(fit - self.best_fit)

            if (self.diffs[0] > 0.001 or self.diffs[1] > 1.0 or self.diffs[2] > 100.) and \
                            len(self.current_fit) > 0:
                self.detected = False  # No fit found
            else:
                self.detected = True
                self.px_count = np.count_nonzero(inds)
                self.current_fit.append(fit)

                if len(self.current_fit) > 5:
                    self.current_fit = self.current_fit[len(self.current_fit) - 5:]

                self.best_fit = np.average(self.current_fit, axis=0)
        else:
            self.detected = False
            if len(self.current_fit) > 0:
                self.current_fit = self.current_fit[:len(self.current_fit) - 1]
            if len(self.current_fit) > 0:
                self.best_fit = np.average(self.current_fit, axis=0)
