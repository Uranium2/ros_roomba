#!/usr/bin/env python3
import predict as pdt
import rospy
from ros_numpy import msgify, numpify
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class Vision:
    def __init__(self):
        self.image_sub = rospy.Subscriber(
            "/rrbot/camera1/image_raw", Image, self.img_cb
        )
        self.image_pub = rospy.Publisher("/stop_sign", Image, queue_size=10)
        self.is_stop_sign = rospy.Publisher("/is_stop_sign", Bool, queue_size=10)
        # Charger le model Tensorflow
        self.model = pdt.Predict()

    def img_cb(self, img):
        image_np = numpify(img)
        image_np, has_stop_sign = self.model.predict_result(image_np)
        self.image_pub.publish(msgify(Image, image_np, encoding="rgb8"))
        self.is_stop_sign.publish(has_stop_sign)
        rospy.loginfo(f"has_stop_sign : {has_stop_sign}")


if __name__ == "__main__":
    rospy.init_node("vision", anonymous=True)
    vision = Vision()
    rospy.spin()
