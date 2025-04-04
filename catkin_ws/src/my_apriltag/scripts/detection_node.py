#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty
import cv2
from pathlib import Path
from sensor_msgs import msg
from cv_bridge import CvBridge
import dt_apriltags as dt



class DetectionNode():

    bridge = CvBridge()

    detector = None
    camera_params = None
    tag_size_meters = 0.2

    def get_camera_params(self):
        result = []
        src_dir = Path(__file__).resolve().parent
        with open(str(src_dir) + '/camera_params.csv', 'r') as f:
            for line in f:
                nums = line.split(',')
                for n in nums:
                    result.append(float(n))
                break
        return result

    def key_listener(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            [i,_,_] = select.select([sys.stdin],[],[],0.1)
            if i:
                return sys.stdin.read(1)
            return None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        detections = self.detector.detect(cv_image,
                        estimate_tag_pose=True,
                        camera_params=self.camera_params,
                        tag_size=self.tag_size_meters)

        for tag in detections:
            print("looks like tag id={}\n".format(tag.tag_id))
            print("pose_R={}\n".format(tag.pose_R))
            print("post_t={}\n".format(tag.pose_t))
            print("pose_err={}\n".format(tag.pose_err))


    def listener(self):
        # Initialize the node
        rospy.init_node('apriltag_detector', anonymous=True)

        # Subscribe to the topic
        rospy.Subscriber('/apriltag_detections', msg.Image, self.callback)


        self.camera_params = self.get_camera_params()
        if (len(self.camera_params) == 0):
            rospy.loginfo("ERROR: Failed to get camera params")
            exit(1)


        self.detector = dt.Detector(families='tag36h11', nthreads=8, debug=0)



        rospy.loginfo("Press q to quit...")

        while not rospy.is_shutdown():
            key = self.key_listener()
            if key == 'q':
                rospy.loginfo("Quiting the program...")
                break

        rospy.loginfo("Shutting Down node...")

if __name__ == '__main__':
    node = DetectionNode()
    node.listener()
