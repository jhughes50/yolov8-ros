#!/usr/bin/python3
"""
    Jason Hughes
    September 2024

    A node to mask a person from an image

    DTC PRONTO 2024
"""
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from ultralytics import YOLO

class PersonMasker:

    def __init__(self):

        self.model_ = YOLO("yolov8m-seg.pt")
        self.bridge_ = CvBridge()
        rospy.Subscriber("/camera/color/image_raw_throttle/compressed", CompressedImage, self.imageCallback)
    
        self.mask_pub_ = rospy.Publisher("/image/masked", Image, queue_size=2)

    def imageCallback(self, msg : CompressedImage) -> None:
        #cv_image = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        results = self.model_(cv_image)

        for r in results:
            boxes = r.boxes
            masks = r.masks

            if boxes is not None:
                for i, box in enumerate(boxes):
                    cid = int(box.cls[0])
                    label = self.model_.names[cid]
                    conf = float(box.conf[0])
                    
                    if cid == 0:
                        mask  = masks[i] if masks is not None else None
                        
                        m_arr = mask.data.cpu().squeeze().numpy()
                        m_arr = cv2.resize(m_arr, (cv_image.shape[1], cv_image.shape[0]))
                        m_arr = (m_arr * 255).astype(np.uint8)
                        
                        m_img = cv2.bitwise_and(cv_image, cv_image, mask=m_arr)
                        
                        m_msg = self.bridge_.cv2_to_imgmsg(m_img, 'bgr8')
                        m_msg.header = msg.header
                        
                        self.mask_pub_.publish(m_msg)
                    else: 
                        continue



if __name__ == "__main__":
    rospy.init_node("masker")
    
    PersonMasker()

    rospy.spin()

