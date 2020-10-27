#!/usr/bin/env python
"""Tool to mask out parts of rosbags."""
import os
from sys import argv

import cv2
from cv_bridge import CvBridge
from rosbag.bag import Bag

# This script can open rosbag files, edit there camera topic by drawing rectangles
# and save the new edited rosbag. The data gets read image by image. To draw a
# rectangle, press the left mouse button in the image and draw where you want to
# have it. Then press double space to see the edited image. If you want to draw
# another rectangle you can do so and get with double space to the edited image.
# If you want to edit the next image press 'q'.
#
# TL;DR: Draw rectangles, press q for next image, double any button to see the edited image

# Drawing function


def draw_circle(event, x, y, flags, param):
    global ix, iy, drawing

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            cv2.rectangle(cv_img, (ix, iy), (x, y), (0, 255, 0), -1)
        else:
            pass
    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        cv2.rectangle(cv_img, (ix, iy), (x, y), (0, 255, 0))


drawing = False
ix, iy = 0, 0
bridge = CvBridge()

# Main functionality
# Rosbags are opened
with Bag(argv[1], "r") as input_bag:
    with Bag(os.path.splitext(argv[1])[0] + "_edited.bag", "w") as outbag:
        # Topics are read
        for topic, msg, t in input_bag.read_messages():
            # If camera topic -> convert to opencv image, and set mousecallback
            if topic == "/camera/image_raw":
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                while True:
                    cv2.namedWindow("image", cv2.WINDOW_NORMAL)
                    cv2.setMouseCallback("image", draw_circle)
                    cv2.imshow("image", cv_img)
                    key = cv2.waitKey(0) & 0xFF
                    # Press q for next images
                    if key == ord("q"):
                        break
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
                img_msg = bridge.cv2_to_imgmsg(cv_img, encoding="passthrough")
                outbag.write("/camera/image_raw", img_msg, t)
            # Else: write the other topics to the new rosbag

            else:
                outbag.write(topic, msg, t)
