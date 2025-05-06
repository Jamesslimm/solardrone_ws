# import rclpy
# from rclpy.node import Node

# class RealsenseSubscriber(Node):
#     def __init__(self):
#         super().__init__('realsense_subscriber_node')
#         self.get_logger().info('RealSense Subscriber Node Started')

# def main(args=None):
#     rclpy.init(args=args)
#     node = RealsenseSubscriber()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# import time

# class RealsenseSubscriber(Node):
#     def __init__(self):
#         super().__init__('realsense_subscriber_node')
#         self.get_logger().info('RealSense Subscriber Node Started')
        
#         self.color_subscription = self.create_subscription(
#             Image, 
#             '/camera/realsense_node/color/image_raw', 
#             self.color_callback, 
#             10)
        
#         self.bridge = CvBridge()
#         self.prev_time = time.time()  # Store the initial time
#         self.fps = 0

#     def color_callback(self, msg):
#         # Convert ROS Image message to OpenCV format
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#         # Calculate FPS
#         curr_time = time.time()
#         self.fps = 1.0 / (curr_time - self.prev_time)
#         self.prev_time = curr_time

#         # Overlay FPS on the frame
#         cv2.putText(cv_image, f"FPS: {self.fps:.2f}", (10, 30), 
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         # Display the frame
#         cv2.imshow("RealSense Color Stream", cv_image)
        
#         # Use waitKey with a small delay to allow OpenCV to process events
#         key = cv2.waitKey(1)
#         if key == 27:  # Press 'ESC' to exit
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     realsense_subscriber = RealsenseSubscriber()
#     rclpy.spin(realsense_subscriber)
#     realsense_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# from cv_bridge import CvBridge
# import time
# import torch
# from ultralytics import YOLO
# from yolov8_msgs.msg import Yolov8Inference, InferenceResult

# class RealsenseSubscriber(Node):
#     def __init__(self):
#         super().__init__('realsense_subscriber_node')
#         self.get_logger().info('RealSense Subscriber Node Started')

#         # Subscription to RealSense color stream
#         self.color_subscription = self.create_subscription(
#             Image,
#             '/camera/realsense_node/color/image_raw',
#             self.color_callback,
#             10
#         )

#         # Publishers
#         self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
#         self.img_pub = self.create_publisher(Image, "/inference_result", 1)

#         # YOLOv8 model initialization
#         self.model = YOLO('/home/james/realsense_ws/src/realsense/model/solar_panel.pt')

#         self.bridge = CvBridge()
#         self.prev_time = time.time()
#         self.fps = 0

#     def color_callback(self, msg):
#         # Convert ROS Image message to OpenCV format
#         cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#         # Calculate FPS
#         curr_time = time.time()
#         self.fps = 1.0 / (curr_time - self.prev_time)
#         self.prev_time = curr_time

#         # Run YOLOv8 inference
#         results = self.model(cv_image)[0]  # YOLOv8 inference on the frame

#         # Process detections
#         for box in results.boxes.data.tolist():
#             x1, y1, x2, y2, conf, cls = box  # Bounding box coordinates, confidence, and class ID
#             label = f"{self.model.names[int(cls)]}: {conf:.2f}"
#             cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
#             cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # Overlay FPS on the frame
#         cv2.putText(cv_image, f"FPS: {self.fps:.2f}", (10, 30),
#                     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#         # Publish inference result as an image
#         inference_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
#         self.img_pub.publish(inference_image_msg)

#         # Publish inference data
#         inference_msg = Yolov8Inference()
#         inference_msg.header.stamp = self.get_clock().now().to_msg()
#         inference_msg = Yolov8Inference()
#         inference_msg.header.stamp = self.get_clock().now().to_msg()

#         inference_msg.yolov8_inference = []  # Ensure it's a list

#         for box in results.boxes.data.tolist():
#             x1, y1, x2, y2, conf, cls = box
#             detection = InferenceResult()
#             detection.class_name = results.names[int(box.cls[0])]  # Set the detected class name
#             detection.left = int(box.xyxy[0][0])  # Use `left` instead of `x1`
#             detection.top = int(box.xyxy[0][1])   # Use `top` instead of `y1`
#             detection.right = int(box.xyxy[0][2]) # Use `right` instead of `x2`
#             detection.bottom = int(box.xyxy[0][3]) # Use `bottom` instead of `y2`
#             detection.confidence = float(conf)
#             detection.class_id = int(cls)

#             inference_msg.yolov8_inference.append(detection)

#         self.yolov8_pub.publish(inference_msg)

#         # Display the frame with YOLOv8 detections
#         cv2.imshow("RealSense YOLOv8 Inference", cv_image)
#         key = cv2.waitKey(1)
#         if key == 27:  # Press 'ESC' to exit
#             rclpy.shutdown()

# def main(args=None):
#     rclpy.init(args=args)
#     realsense_subscriber = RealsenseSubscriber()
#     rclpy.spin(realsense_subscriber)
#     realsense_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import Header
# from geometry_msgs.msg import Twist, Vector3
# from yolov8_msgs.msg import Yolov8Inference, InferenceResult  # Custom message types
# import cv2
# from cv_bridge import CvBridge
# import torch
# from ultralytics import YOLO

# class RealSenseSubscriber(Node):
#     def __init__(self):
#         super().__init__('realsense_subscriber_node')

#         # Initialize OpenCV bridge
#         self.bridge = CvBridge()

#         # Load YOLOv8 model
#         self.model = YOLO('/home/james/realsense_ws/src/realsense/model/solar_panel.pt')  # YOLOv8 Segmentation Model

#         # ROS2 Subscribers
#         self.color_sub = self.create_subscription(
#             Image, 
#             '/camera/realsense_node/color/image_raw', 
#             self.color_callback, 
#             10)

#         # ROS2 Publishers
#         self.inference_pub = self.create_publisher(Yolov8Inference, '/yolov8/detections', 10)
#         self.img_pub = self.create_publisher(Image, "/yolov8/annotated_image", 10)
#         self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

#         # Movement control variables
#         self.counter = 0
#         self.counter_limit = 10
#         self.distance_front = False

#     def color_callback(self, msg):
#         """Processes incoming image, runs YOLOv8 segmentation, and publishes results."""
#         try:
#             # Convert ROS2 Image to OpenCV format
#             img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#             # Run YOLOv8 inference
#             results = self.model(img)

#             # Frame dimensions
#             frame_width = img.shape[1]
#             frame_height = img.shape[0]
#             center_region_width = 40  # Central region width
#             center_region_height = 60  # Central region height

#             # Initialize inference message
#             inference_msg = Yolov8Inference()
#             inference_msg.header.stamp = self.get_clock().now().to_msg()
#             inference_msg.header.frame_id = "inference"

#             for r in results:
#                 masks = r.masks  # Segmentation masks
#                 confidences = r.boxes.conf

#                 if masks is not None:
#                     for mask, confidence in zip(masks.data, confidences):

#                         if confidence < 0.5:  # 🚀 Filter out detections below 0.5 confidence
#                             continue  

#                         # Compute centroid of mask
#                         mask_np = mask.cpu().numpy().astype('uint8')
#                         M = cv2.moments(mask_np)
#                         if M['m00'] != 0:
#                             center_x = int(M['m10'] / M['m00'])
#                             center_y = int(M['m01'] / M['m00'])

#                             # Movement control logic
#                             linear_vec = Vector3()
#                             angular_vec = Vector3()

#                             # Define central region
#                             central_x_min = (frame_width - center_region_width) // 2
#                             central_x_max = (frame_width + center_region_width) // 2
#                             central_y_min = (frame_height - center_region_height) // 2
#                             central_y_max = (frame_height + center_region_height) // 2

#                             if center_x < central_x_min:
#                                 linear_vec.y = 0.2  # Move left
#                             elif center_x > central_x_max:
#                                 linear_vec.y = -0.2  # Move right

#                             if center_y < central_y_min:
#                                 linear_vec.z = 0.2  # Move up
#                             elif center_y > central_y_max:
#                                 linear_vec.z = -0.2  # Move down

#                             if (central_x_min <= center_x <= central_x_max and 
#                                 central_y_min <= center_y <= central_y_max):
#                                 self.counter += 1
#                                 if self.counter >= self.counter_limit:
#                                     self.distance_front = True
#                                     self.counter = 0

#                             # Publish movement command
#                             twist = Twist(linear=linear_vec, angular=angular_vec)
#                             self.cmd_vel_publisher.publish(twist)

#             # Annotate frame with detection results
#             annotated_frame = results[0].plot()
#             img_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")

#             # Publish annotated image and inference results
#             self.img_pub.publish(img_msg)
#             self.inference_pub.publish(inference_msg)

#         except Exception as e:
#             self.get_logger().error(f"Error processing image: {str(e)}")

# def main(args=None):
#     """ROS2 node entry point."""
#     rclpy.init(args=args)
#     realsense_subscriber = RealSenseSubscriber()
#     rclpy.spin(realsense_subscriber)
#     realsense_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from yolov8_msgs.msg import Yolov8Inference, InferenceResult
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class RealSenseSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_subscriber_node')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/home/james/realsense_ws/src/realsense/model/solar_panel.pt')

        # ROS2 Subscribers
        self.color_sub = self.create_subscription(
            Image, 
            '/camera/realsense_node/color/image_raw', 
            self.color_callback, 
            10)

        # ROS2 Publishers
        self.inference_pub = self.create_publisher(Yolov8Inference, '/yolov8/detections', 10)
        self.img_pub = self.create_publisher(Image, "/yolov8/annotated_image", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/offboard_velocity_cmd", 10)
        self.arm_publisher = self.create_publisher(Bool, "/arm_message", 10)

        # Movement control variables
        self.counter = 0
        self.counter_limit = 10
        self.distance_front = False

        # Automatically arm the drone when the node starts
        self.arm_drone()

    def arm_drone(self):
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_publisher.publish(arm_msg)
        self.get_logger().info("Drone armed.")

    def color_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(img)

            frame_width = img.shape[1]
            frame_height = img.shape[0]
            center_region_width = 300  # 300 px width threshold
            center_region_height = 100  # 100 px height threshold

            inference_msg = Yolov8Inference()
            inference_msg.header.stamp = self.get_clock().now().to_msg()
            inference_msg.header.frame_id = "inference"

            # Define center bounding box region
            central_x_min = (frame_width - center_region_width) // 2
            central_x_max = (frame_width + center_region_width) // 2
            central_y_min = (frame_height - center_region_height) // 2
            central_y_max = (frame_height + center_region_height) // 2

            # Draw center bounding box (red rectangle)
            cv2.rectangle(img, (central_x_min, central_y_min), (central_x_max, central_y_max), (0, 0, 255), 2)

            # Variables to track global bounding box
            x_min, y_min, x_max, y_max = float("inf"), float("inf"), float("-inf"), float("-inf")
            object_detected = False  # Track if any object is detected

            for r in results:
                masks = r.masks
                confidences = r.boxes.conf
                boxes = r.boxes.xyxy  # Bounding boxes (xyxy format)

                if masks is not None and boxes is not None:
                    for confidence, box in zip(confidences, boxes):
                        if confidence < 0.5:
                            continue

                        object_detected = True  # At least one object is detected

                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.tolist())

                        # Expand global bounding box to include this box
                        x_min = min(x_min, x1)
                        y_min = min(y_min, y1)
                        x_max = max(x_max, x2)
                        y_max = max(y_max, y2)

            if object_detected:
                # Compute the fused center point
                bbox_center_x = (x_min + x_max) // 2
                bbox_center_y = (y_min + y_max) // 2

                print(f"Fused Bounding Box: ({x_min}, {y_min}) to ({x_max}, {y_max})")
                print(f"Fused Center Point: ({bbox_center_x}, {bbox_center_y})")

                # Draw the fused bounding box (blue)
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 0, 0), 2)

                # Draw the fused center point (red circle)
                cv2.circle(img, (bbox_center_x, bbox_center_y), 5, (0, 0, 255), -1)

                # Movement logic based on the fused center
                linear_vec = Vector3()
                angular_vec = Vector3()

                if bbox_center_x < central_x_min:
                    linear_vec.y = 1.0  # Move left
                elif bbox_center_x > central_x_max:
                    linear_vec.y = -1.0  # Move right

                if bbox_center_y < central_y_min:
                    linear_vec.z = 1.0  # Move up
                elif bbox_center_y > central_y_max:
                    linear_vec.z = -1.0  # Move down

                twist = Twist(linear=linear_vec, angular=angular_vec)
                self.cmd_vel_publisher.publish(twist)

            # Image publishing
            annotated_frame = results[0].plot()

            # Overlay bounding box and center marker on the final annotated image
            final_image = cv2.addWeighted(annotated_frame, 0.8, img, 0.2, 0)

            img_msg = self.bridge.cv2_to_imgmsg(final_image, encoding="bgr8")
            self.img_pub.publish(img_msg)
            self.inference_pub.publish(inference_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    realsense_subscriber = RealSenseSubscriber()
    rclpy.spin(realsense_subscriber)
    realsense_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
