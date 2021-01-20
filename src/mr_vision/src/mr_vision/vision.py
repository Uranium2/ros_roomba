#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from ros_numpy import numpify, msgify
import rospkg
import os
import ros_numpy

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
        self.is_stop_sign = rospy.Publisher("/is_stop_sign", Bool)
        # Charger le model Tensforflow
        self.predict = pdt.Predict()


    def img_cb(self, img):
        
        image_np = numpify(img)
        image_np, has_stop_sign = self.predict.predict_result(image_np)


        self.image_pub.publish(msgify(Image, image_np, encoding='rgb8'))
        self.is_stop_sign.publish(has_stop_sign)
        rospy.loginfo("has_stop_sign : " + str(has_stop_sign))

if __name__ == '__main__':
    rospy.init_node('vision', anonymous=True)
    vision = Vision()
    rospy.spin()
