#!/usr/bin/env python3
import math

import rospy
import cv2
import torch
import torch.backends.cudnn as cudnn
import numpy as np
from cv_bridge import CvBridge
from pathlib import Path
import os
import sys
from rostopic import get_topic_type

from sensor_msgs.msg import Image, CompressedImage
from detection_msgs.msg import BoundingBox, BoundingBoxes


# add yolov5 submodule to path
from yolov5_ros.src.yolov5.utils.general import xyxy2xywh

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0] / "yolov5"
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative path

# import from yolov5 submodules
from models.common import DetectMultiBackend
from utils.general import (
    check_img_size,
    check_requirements,
    non_max_suppression,
    scale_coords
)
from utils.plots import Annotator, colors
from utils.torch_utils import select_device
from utils.augmentations import letterbox


@torch.no_grad()
class Yolov5Detector:
    def __init__(self):
        self.kernel_size = 5  # 5 - 7
        self.low_threshold = 50
        self.high_threshold = 150
        self.rho = 1  # distance resolution in pixels of the Hough grid
        self.theta = np.pi / 180  # angular resolution in radians of the Hough grid
        self.threshold = 90  # minimum number of votes (intersections in Hough grid cell)
        self.min_line_length = 80  # minimum number of pixels making up a line
        self.max_line_gap = 15  # maximum gap in pixels between connectable line segments
        self.conf_thres = rospy.get_param("~confidence_threshold")
        self.iou_thres = rospy.get_param("~iou_threshold")
        self.agnostic_nms = rospy.get_param("~agnostic_nms")
        self.max_det = rospy.get_param("~maximum_detections")
        self.classes = rospy.get_param("~classes", None)
        self.line_thickness = rospy.get_param("~line_thickness")
        self.view_image = rospy.get_param("~view_image")
        # Initialize weights 
        weights = rospy.get_param("~weights")
        # Initialize model
        self.device = select_device(str(rospy.get_param("~device","")))
        self.model = DetectMultiBackend(weights, device=self.device, dnn=rospy.get_param("~dnn"), data=rospy.get_param("~data"))
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = (
            self.model.stride,
            self.model.names,
            self.model.pt,
            self.model.jit,
            self.model.onnx,
            self.model.engine,
        )

        # Setting inference size
        self.img_size = [rospy.get_param("~inference_size_w", 640), rospy.get_param("~inference_size_h",480)]
        self.img_size = check_img_size(self.img_size, s=self.stride)

        # Half
        self.half = rospy.get_param("~half", False)
        self.half &= (
            self.pt or self.jit or self.onnx or self.engine
        ) and self.device.type != "cpu"  # FP16 supported on limited backends with CUDA
        if self.pt or self.jit:
            self.model.model.half() if self.half else self.model.model.float()
        bs = 1  # batch_size
        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model.warmup()  # warmup        
        
        # Initialize subscriber to Image/CompressedImage topic
        input_image_type, input_image_topic, _ = get_topic_type(rospy.get_param("~input_image_topic"), blocking = True)
        self.compressed_input = input_image_type == "sensor_msgs/CompressedImage"

        if self.compressed_input:
            self.image_sub = rospy.Subscriber(
                input_image_topic, CompressedImage, self.callback, queue_size=1
            )
        else:
            self.image_sub = rospy.Subscriber(
                input_image_topic, Image, self.callback, queue_size=1
            )

        # Initialize prediction publisher
        self.pred_pub = rospy.Publisher(
            rospy.get_param("~output_topic"), BoundingBoxes, queue_size=10
        )
        # Initialize image publisher
        self.publish_image = rospy.get_param("~publish_image")
        if self.publish_image:
            self.image_pub = rospy.Publisher(
                rospy.get_param("~output_image_topic"), Image, queue_size=10
            )
        
        # Initialize CV_Bridge
        self.bridge = CvBridge()

    def callback(self, data):
        """adapted from yolov5/detect.py"""
        # print(data.header)
        if self.compressed_input:
            im = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding="bgr8")
        else:
            im = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        im, im0 = self.preprocess(im)
        # print(im.shape)
        # print(img0.shape)
        # print(img.shape)

        # Run inference
        im = torch.from_numpy(im).to(self.device) 
        im = im.half() if self.half else im.float()
        im /= 255
        if len(im.shape) == 3:
            im = im[None]

        pred = self.model(im, augment=False, visualize=False)
        pred = non_max_suppression(
            pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det
        )

        gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
        imc = im0.copy() #if save_crop else im0  # for save_crop
        fimc = np.copy(imc) * 0

        # Process predictions 
        det = pred[0].cpu().numpy()

        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = data.header
        bounding_boxes.image_header = data.header
        
        annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

            #
            # Write results
            for *xyxy, conf, cls in reversed(det):
                #if save_txt:  # Write to file
                    xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                    line = (cls, *xywh, conf)# if save_conf else (cls, *xywh)  # label format
                  #  with open(f'{txt_path}.txt', 'a') as f:
                   #     f.write(('%g ' * len(line)).rstrip() % line + '\n')

                # if self.save_img or self.save_crop or self.view_img:  # Add bbox to image
                    c = int(cls)  # integer class

                    label = f"{self.names[c]} {conf:.2f}"
                    # coords of bounding box
                    x1 = int(xyxy[0])
                    y1 = int(xyxy[1])
                    x2 = int(xyxy[2])
                    y2 = int(xyxy[3])
                    h = y2 - y1  # height of box
                    w = x2 - x1  # width of box

                    # convert img to gray for gamma correction
                    grayG = cv2.cvtColor(imc, cv2.COLOR_BGR2GRAY)

                    # compute gamma = log(mid*255)/log(mean)
                    mid = 0.5
                    mean = np.mean(grayG)
                    gamma = math.log(mid * 255) / math.log(mean)

                    # do gamma correction
                    imcg = np.power(imc, gamma).clip(0, 255).astype(np.uint8)

                    # Set up things for OpenCV to be annoying with. Seriously, I hate the difference in channels with the alpha channel
                    roi = imcg[y1:y1 + h, x1:x1 + w]
                    final = imc[y1:y1 + h, x1:x1 + w] * 0

                    # convert to gray
                    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    # perform gaussian blur to make blur gray image
                    gray = cv2.GaussianBlur(gray, (23, 23), 3)
                    # canny image
                    gray = cv2.Canny(gray, self.low_threshold, self.high_threshold, 3)
                    # for bounding box
                    roi = np.zeros_like(roi)
                    roi[:, :, 0] = gray
                    roi[:, :, 1] = gray
                    roi[:, :, 2] = gray

                    # Run Hough on edge detected image
                    # Output "lines" is an array containing endpoints of detected line segments
                    lines = cv2.HoughLinesP(gray, self.rho, self.theta, self.threshold, np.array([]),
                                            self.min_line_length, self.max_line_gap)
                    # double null check and draw lines
                    if lines is not None:
                        if len(lines):
                            for line in lines:
                                for xl1, yl1, xl2, yl2 in line:
                                    cv2.line(final, (xl1, yl1), (xl2, yl2), (255, 255, 255), 5)

                    # place lines
                    fimc[y1:y1 + roi.shape[0], x1:x1 + roi.shape[1]] = final

            ### POPULATE THE DETECTION MESSAGE HERE

            # Stream results
            im0 = annotator.result()

        # Publish prediction
        self.pred_pub.publish(bounding_boxes)

        # Publish & visualize images
        if self.view_image:
            cv2.imshow(str(0), im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.publish_image:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(im0, "bgr8"))
        

    def preprocess(self, img):
        """
        Adapted from yolov5/utils/datasets.py LoadStreams class
        """
        img0 = img.copy()
        img = np.array([letterbox(img, self.img_size, stride=self.stride, auto=self.pt)[0]])
        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        return img, img0 


if __name__ == "__main__":

    check_requirements(exclude=("tensorboard", "thop"))
    
    rospy.init_node("yolov5", anonymous=True)
    detector = Yolov5Detector()
    
    rospy.spin()
