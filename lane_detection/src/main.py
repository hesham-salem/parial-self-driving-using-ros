import cv2
import numpy as np
from detector import LaneDepartureDetector


def create_config(height, width):
    center_x = 670
    center_y = height / 2
    x_top_factor = 0.04
    x_lower_factor = 0.5
    lower_left = [center_x - x_lower_factor * width, height]
    lower_right = [center_x + x_lower_factor * width, height]
    top_left = [center_x - x_top_factor * width, center_y + height / 10]
    top_right = [center_x + x_top_factor * width, center_y + height / 10]

    roi_matrix = np.int32([
        lower_left, top_left, top_right, lower_right
    ])

    src = np.float32([(575, 464),
                      (707, 464),
                      (258, 682),
                      (1049, 682)])

    dst = np.float32([(550, 0),
                      (width - 500, 0),
                      (500, height),
                      (width - 550, height)])

    return src, dst, roi_matrix


path = "/home/hesham/my_ws2/src/lane_detection/src/lanes.mp4"
video = cv2.VideoCapture(path)
counter = 0
if video.isOpened():
    width = video.get(3)
    height = video.get(4)
    (src, dst, roi) = create_config(height, width)
    detector = LaneDepartureDetector(src, dst, roi)

    while video.isOpened():
        ret, image = video.read()

        if image is None:
            break

        # if counter % 10 == 0:
        #     cv2.imwrite("./out/before" + str(counter) + str(".jpg"), image)

        (final_image, departure) = detector.process_image(image)
        cv2.imshow('Departure Warning', final_image)

        # if counter % 10 == 0:
        #     cv2.imwrite("./out/after" + str(counter) + str(".jpg"), final_image)

        counter += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
video.release()
