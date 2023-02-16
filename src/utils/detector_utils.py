# from pupper_bringup.msg import Box, Boxes
from coco_labels import COCO_LABELS
import cv2



def draw_boxes(img, output):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (255, 0, 0)
    thickness = 1
    for out in output:
        bbox = out.bbox
        label = COCO_LABELS[out.id+1]
        img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
        org = (bbox.xmin, bbox.ymin)
        img = cv2.putText(img, label, org, font, 
        fontScale, color, thickness, cv2.LINE_AA)

    return img