#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from ros_numpy import numpify, msgify
import rospkg
import os
import pathlib

import numpy as np
import tensorflow as tf
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as viz_utils

import predict as pdt



class Vision:
    def __init__(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('mr_vision') # path is the path to the package mr_vision
        self.image_sub = rospy.Subscriber('/rrbot/camera1/image_raw', Image, self.img_cb)
        self.image_pub = rospy.Publisher('/stop_sign', Image)
        # Charger le model Tensforflow
        self.predict = pdt.Predict()


    def img_cb(self, img):
        rospy.loginfo('received an image')
        image_np = numpify(img)
        image_np = self.predict.predict_result(image_np)
        #TODO detect stop sign and add Bounding box to the published image_np

        self.image_pub.publish(msgify(Image, image_np, encoding='rgb8'))

if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    vision = Vision()
    rospy.spin()
