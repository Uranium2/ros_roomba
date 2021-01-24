#!/usr/bin/env python3
from collections import deque

import predict as pdt
import rospy
from ros_numpy import msgify, numpify
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class Vision:
    def __init__(self, stop_sign_occurrences: int = 3):
        self.image_sub = rospy.Subscriber(
            "/rrbot/camera1/image_raw", Image, self.img_cb
        )
        self.image_pub = rospy.Publisher("/stop_sign", Image, queue_size=10)
        self.is_stop_sign = rospy.Publisher("/is_stop_sign", Bool, queue_size=10)
        # Charger le model Tensorflow
        self.model = pdt.Predict()
        self.stop_sign_deque = deque(maxlen=stop_sign_occurrences)

    def img_cb(self, img):
        image_np = numpify(img)
        image_np, has_stop_sign = self.model.predict_result(image_np)
        vcss = self.video_contains_stop_sign(has_stop_sign)
        self.image_pub.publish(msgify(Image, image_np, encoding="rgb8"))
        self.is_stop_sign.publish(vcss)
        rospy.loginfo(f"Video contains stop sign : {vcss}")

    def video_contains_stop_sign(self, has_stop_sign: bool) -> bool:
        self.stop_sign_deque.append(has_stop_sign)
        return self.stop_sign_deque.maxlen == self.stop_sign_deque.count(True)


if __name__ == "__main__":
    rospy.init_node("vision", anonymous=True)
    vision = Vision()
    rospy.spin()
