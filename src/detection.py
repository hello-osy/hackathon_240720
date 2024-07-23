#!/usr/bin/env python

import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge

class YoloDetector:
    def __init__(self):
        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='path/to/your/best.pt')
        # ROS 이미지 메시지와 OpenCV 이미지 간 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()
        # /camera/image_raw 토픽 구독
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        # /detected_objects 토픽 발행
        self.result_pub = rospy.Publisher('/detected_objects', Int32MultiArray, queue_size=10)

    def image_callback(self, data):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        # YOLOv5 모델을 사용하여 객체 검출
        results = self.model(cv_image)
        # 검출 결과를 NumPy 배열로 변환
        detections = results.xyxy[0].cpu().numpy()

        # Int32MultiArray 메시지 초기화
        detected_objects_msg = Int32MultiArray()

        # 검출된 각 객체에 대해 반복
        for detection in detections:
            x1, y1, x2, y2, conf, cls_id = detection  # 경계 상자 좌표와 클래스 ID, 신뢰도
            width = int(x2 - x1)
            height = int(y2 - y1)
            # Int32 배열에 클래스 ID, 가로 길이, 세로 길이 추가
            detected_objects_msg.data.extend([int(cls_id), width, height])

        # 객체 정보를 퍼블리시
        self.result_pub.publish(detected_objects_msg)

if __name__ == '__main__':
    # ROS 노드 초기화
    rospy.init_node('yolo_detector', anonymous=True)
    # YoloDetector 클래스 인스턴스 생성
    yolo_detector = YoloDetector()
    try:
        # ROS 이벤트 루프 실행
        rospy.spin()
    except KeyboardInterrupt:
        # 노드 종료 시 메시지 출력
        rospy.loginfo("Shutting down YOLO Detector Node")
