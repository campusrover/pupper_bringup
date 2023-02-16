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

import os

# dir = os.path.abspath(os.getcwd()) 

class MobilenetDetector():

  def __init__(self, detection_key: str):
    self.key = detection_key

    self.interpreter = tflite.Interpreter("src/models/tf2_ssd_mobilenet_v2_coco17_ptq_edgetpu.tflite",
      experimental_delegates=[tflite.load_delegate('libedgetpu.so.1')])

    self.interpreter.allocate_tensors()
    self.size = common.input_size(self.interpreter)

    self.make_label_dict()
    self.pub = rospy.Publisher("/boxes", CompressedImage, queue_size = 10)
    self.img_sub = rospy.Subscriber("/image/compressed", CompressedImage, self.img_cb)

  def img_cb(self, msg):
    np_arr = imgmsg_to_numpy(msg)
    output, np_arr =  self.run_model(np_arr)

    boxes = self.draw_boxes(np_arr, output)

    print(boxes)
    self.pub.publish(numpy_to_imgmsg(boxes))


  def make_label_dict(self):
    with open("src/labels/coco_labels.txt") as f:
      self.label_dict = {}
      lines = f.readlines()

      for line in lines:
        split = line.split()
        id, label = split[0], split[1]
        self.label_dict[int(id)] = label


  def draw_boxes(self, img, output):
    font = cv2.FONT_HERSHEY_SIMPLEX
    fontScale = 1
    color = (255, 0, 0)
    thickness = 1
    for out in output:
      bbox = out.bbox
      print('hello')
      label = self.label_dict[out.id]
      print(label)
      img = cv2.rectangle(img, (bbox.xmin, bbox.ymin), (bbox.xmax, bbox.ymax), (255,0,0), 2)
      org = (bbox.xmin, bbox.ymin)
      img = cv2.putText(img, label, org, font, 
        fontScale, color, thickness, cv2.LINE_AA)

    return img

  def run_model(self, image):

    img = im.fromarray(image).convert('RGB').resize(self.size, im.ANTIALIAS)
    
    common.set_input(self.interpreter, img)
    self.interpreter.invoke()

    return detect.get_objects(self.interpreter, score_threshold=0.5), np.array(img)


if __name__ == "__main__":
  # print(dir)
  rospy.init_node("detector")
  rate = rospy.Rate(10)
  detector = MobilenetDetector("person")
  rospy.spin()




    