import cv2
import numpy as np
from detector import LaneDepartureDetector
from moviepy.editor import VideoFileClip


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


(src, dst, roi) = create_config(720, 1280)
detector = LaneDepartureDetector(src, dst, roi)

video_output1 = './out/lanes_processed.mp4'
video_input1 = VideoFileClip('lanes.mp4')
processed_video = video_input1.fl_image(detector.process_image)
processed_video.write_videofile(video_output1, audio=False)
