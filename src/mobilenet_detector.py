#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage

import pycoral.adapters.common as common
import pycoral.adapters.detect as detect
import tflite_runtime.interpreter as tflite

from PIL import Image as im
import numpy as np
import cv2
from utils.bridge import numpy_to_imgmsg, imgmsg_to_numpy
from labels.coco_labels import COCO_LABELS

import os

# dir = os.path.abspath(os.getcwd()) 

class MobilenetDetector():

  def __init__(self, detection_key: str):
    self.key = detection_key

    self.interpreter = tflite.Interpreter("src/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite",
      experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

    self.interpreter.allocate_tensors()
    self.size = common.input_size(self.interpreter)

    self.label_dict = COCO_LABELS
    self.pub = rospy.Publisher("/boxes/compressed", CompressedImage, queue_size = 10)
    self.img_sub = rospy.Subscriber("/image/compressed", CompressedImage, self.img_cb)

    self.img = None

  def img_cb(self, msg):
    self.img = np_arr = imgmsg_to_numpy(msg)
    

  def draw_boxes(self, img, output):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (255, 0, 0)
    thickness = 1
    for out in output:
      bbox = out.bbox
      label = self.label_dict[out.id+1]
      img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
      org = (bbox.xmin, bbox.ymin)
      img = cv2.putText(img, label, org, font, 
        fontScale, color, thickness, cv2.LINE_AA)

    return img

  def run_model(self):
    if self.img is not None:
      img = im.fromarray(self.img).convert('RGB').resize(self.size, im.ANTIALIAS)
      
      common.set_input(self.interpreter, img)
      self.interpreter.invoke()

      output, np_arr =  detect.get_objects(self.interpreter, score_threshold=0.5), np.array(img)

      boxes = self.draw_boxes(np_arr, output)

      self.pub.publish(numpy_to_imgmsg(boxes))

  


if __name__ == "__main__":
  # print(dir)
  rospy.init_node("detector")
  rate = rospy.Rate(10)
  detector = MobilenetDetector("person")
  while not rospy.is_shutdown():
    rate.sleep()
    detector.run_model()




    