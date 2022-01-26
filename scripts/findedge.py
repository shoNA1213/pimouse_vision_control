#!/usr/bin/env python
#encording: utf8
import rospy, cv2
#import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class FaceToFace():
    def __init__(self):
        sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.get_image)
        self.bridge = CvBridge()
        self.image_org = None
        self.pub = rospy.Publisher("face", Image, queue_size=1)

    def get_image(self,img):
        try:
            self.image_org = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

    def monitor(self,r,org2):
        
        gray = cv2.cvtColor(org2, cv2.COLOR_BGR2GRAY)
        ret, th = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
        edge = cv2.Canny(th, 100, 200, apertureSize = 3)
        self.pub.publish(self.bridge.cv2_to_imgmsg(edge, "mono8"))

    def detect_face(self):
        if self.image_org is None:
            return None

        org = self.image_org
        height = org.shape[0]
        width = org.shape[1]
        center = (int(width/2), int(height/2))
        angle = 180.0
        scale = 1.0
        trans = cv2.getRotationMatrix2D(center, angle, scale)
        org2 = cv2.warpAffine(org, trans, (width,height))

        gimg = cv2.cvtColor(org2,cv2.COLOR_BGR2GRAY)
        classifier = "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"
        cascade = cv2.CascadeClassifier(classifier)
        face = cascade.detectMultiScale(gimg,1.1,1,cv2.CASCADE_FIND_BIGGEST_OBJECT)

        if len(face) == 0:
            self.monitor(None,org2)
            return None

        r = face[0]
        self.monitor(r,org2)
        return r

if __name__ == '__main__':
    rospy.init_node('face_to_face')
    fd = FaceToFace()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo(fd.detect_face())
        rate.sleep()
