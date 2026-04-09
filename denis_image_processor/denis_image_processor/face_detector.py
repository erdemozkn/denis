#!/usr/bin/python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FaceDetector(Node):
    def __init__(self):
        super().__init__("face_detector")

        model_path = os.path.join(
            os.path.dirname(__file__), 
            "face_detection_yunet_2023mar.onnx"
        )
        
        if not os.path.exists(model_path):
            self.get_logger().error(f"Model bulunamadı! Yol: {model_path}")
            raise FileNotFoundError(f"YuNet modeli eksik: {model_path}")

        self.bridge = CvBridge()

        self.face_detector = cv2.FaceDetectorYN.create(
            model=model_path,
            config="",
            input_size=(320, 320),
            score_threshold=0.4,
            nms_threshold=0.3,
            top_k=5000
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(Image, '/camera/face_detected', 10)

        self.get_logger().info('YuNet Yüz tespit node başlatıldı! (Daha stabil ve hızlı)')

    def image_callback(self, msg):
        try:
            # ROS → OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Performans için görüntüyü küçült (Raspberry Pi önerisi)
            h, w = frame.shape[:2]
            if w > 640:
                scale = 640 / w
                frame_small = cv2.resize(frame, None, fx=scale, fy=scale)
            else:
                frame_small = frame

            # YuNet beklediği boyuta ayarla
            self.face_detector.setInputSize((frame_small.shape[1], frame_small.shape[0]))

            # Yüz tespiti
            faces = self.face_detector.detect(frame_small)

            if faces[1] is not None:
                for face in faces[1]:
                    # Koordinatlar
                    x, y, width, height = face[0:4].astype(int)
                    confidence = face[-1]

                    # Dikdörtgen çiz
                    cv2.rectangle(frame_small, (x, y), (x + width, y + height), (0, 255, 0), 3)

                    # Güven skoru yaz
                    cv2.putText(frame_small, f'{int(confidence*100)}%', 
                                (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # İşlenmiş görüntüyü yayınla
            ros_image = self.bridge.cv2_to_imgmsg(frame_small, encoding="bgr8")
            self.publisher.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f'Hata: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()