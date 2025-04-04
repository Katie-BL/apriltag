import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs import msg

#import apriltag
from apriltag import Detector

camera = cv2.VideoCapture(0)
bridge = CvBridge()

if not camera.isOpened():
    print("Error")
    exit()

def node():
    rospy.init_node('camera_demo', anonymous=True)
    detection_pub = rospy.Publisher('/apriltag_detections', msg.Image, queue_size=10)

    while not rospy.is_shutdown():
        ret, frame = camera.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_message = bridge.cv2_to_imgmsg(gray, encoding="passthrough")
        detection_pub.publish(image_message)
        # Wait for key press (exit on 'q')
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    node()

# https://robotics.stackexchange.com/questions/80961/how-to-install-cv-bridge
# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPythonnROSImagesAndOpenCVImagesPython