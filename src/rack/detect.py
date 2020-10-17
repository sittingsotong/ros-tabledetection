import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rosbag
import os 

class table_detector:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic", Image)
        self.bridge = CvBridge()

        self.rate = rospy.Rate(10)

    def run(self, path, topic):
        bag = rosbag.Bag(path)
        # bag = rosbag.Bag('../../data/rack_test/2019-02-27-19-35-19.bag')

        params = cv2.SimpleBlobDetector_Params()

        params.filterByColor = True

        params.filterByArea = True
        params.minArea = 1000

        detector = cv2.SimpleBlobDetector_create(params)

        for topic, msg, t in bag.read_messages(topics=topic):
            try:
                im = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except CvBridgeError as e:
                print(e)

            cv2.imshow("Image", im)

            masked = self.colour_filter(im)
            
            gray = self.convert_to_gray(masked)
            result = self.leg_detection(gray)
            
            cv2.imshow("Final", result)
            cv2.waitKey(0)

            try: 
                img_msg = self.bridge.cv2_to_imgmsg(result, encoding='passthrough')
                self.image_pub.publish(img_msg)
                self.rate.sleep()
            except CvBridgeError as e:
                print(e)
        
        bag.close()


    def colour_filter(self, im):
        # filtering by colour
            lower = np.array([110, 25, 25], dtype='uint8')
            upper = np.array([220, 88, 40], dtype='uint8')

            mask = cv2.inRange(im, lower, upper)
            masked = cv2.bitwise_and(im, im, mask=mask)

            return masked

    def convert_to_gray(self, im):
        if len(im.shape) != 2:
            gray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        else:
            gray = im
        
        return gray

    def leg_detection(self, im):
        # invert colours to black bg
        invert = cv2.bitwise_not(im)

        # blur image to make extracting edges easier
        blur = cv2.medianBlur(invert, 25)
        blur = cv2.GaussianBlur(blur, (3,3), 0)
        bw = cv2.adaptiveThreshold(blur, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 15, -2)

        vertical = np.copy(bw)

        # extract vertical lines
        open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 58))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))

        # close away noise in object then remove noise outside
        vertical = cv2.morphologyEx(vertical, cv2.MORPH_CLOSE, close_kernel)
        vertical = cv2.morphologyEx(vertical, cv2.MORPH_OPEN, open_kernel)

        # invert colours back to black lines
        vertical = cv2.bitwise_not(vertical)

        # get edges
        edges = cv2.adaptiveThreshold(vertical, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, -2)

        # smooth image
        kernel = np.ones((2,2), np.uint8)
        edges = cv2.dilate(edges, kernel)
        smooth = np.copy(vertical)
        smooth = cv2.blur(smooth, (2,2))
        (rows, cols) = np.where(edges != 0)
        vertical[rows, cols] = smooth[rows, cols]

        return vertical

def main():
    rospy.init_node('detect')
    td = table_detector() 

    path = rospy.get_param("detect/file_path")
    topic = ['mono_front/usb_cam/image_rect_color']
    
    td.run(path, topic)

    rospy.spin()

