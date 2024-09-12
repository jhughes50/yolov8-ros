#!/usr/bin/python3
"""
    Jason Hughes
    September 2024

    A node to mask a person from an image

    DTC PRONTO 2024
"""
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

class PersonMasker:

    def __init__(self):

        self.model_ = YOLO("yolov8n-seg.pt")
        self.bridge_ = CvBridge()
        rospy.Subscriber("/camera/color/image_raw", Image, self.imageCallback)
    
        self.mask_pub_ = "/image/masked", Image, queue_size=2)

    def imageCallback(self, msg : Image) -> None:
        cv_image = self.bridge_.imgmsg_to_cv2(msg, "bgr8")

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
                        mask = masks[i] if masks is not None else None
                        m_arr = mask.data.cpu().numpy()
                        m_img = cv2.bitwise_and(cv_image, m_arr)
                        
                        m_msg = self.bridge_.cv2_to_imgmsg(m_img, 'bgr8')
                        m_msg.header = msg.header
                        self.mask_pub_.publish(m_msg)
                    else: 
                        continue



if __name__ == "__main__":
    rospy.init_node("masker")
    
    PersonMasker()

    rospy.spin()

