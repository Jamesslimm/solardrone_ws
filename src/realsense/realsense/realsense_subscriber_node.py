#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import math
import numpy as np
import logging
from cv_bridge import CvBridge
# logging.getLogger('ultralytics').setLevel(logging.ERROR)
from geometry_msgs.msg import Point
from std_msgs.msg import Float64, Float32MultiArray
import matplotlib.pyplot as plt
import open3d as o3d 
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2
from yolov8_msgs.msg import Yolov8Inference
import cv2
from ultralytics import YOLO
import os
from datetime import datetime
import pandas as pd

class ZDCalNode(Node):
    def __init__(self):
        super().__init__('realsense_subscriber_node')

        # Initialize OpenCV bridge
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = YOLO('/home/crestjj/realsense_ws/src/realsense/model/solar_panel.pt')

        # Define QoS profile for PX4 compatibility
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.inference_pub = self.create_publisher(Yolov8Inference, '/yolov8/detections', 10)
        self.img_pub = self.create_publisher(Image, "/yolov8/annotated_image", 10)

        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        self.drop_point_pub = self.create_publisher(Point, 'drop_point', 10)
        self.inclination_pub = self.create_publisher(Float64, 'solar_panel_inclination' , 10)

        # ROS2 Subscribers
        self.color_sub = self.create_subscription(
            Image, 
            '/camera/realsense_node/color/image_raw', #'/camera/realsense_node/color/image_raw', 
            self.color_callback, 
            10)
    
        self.depth_sub = self.create_subscription(
            PointCloud2,                              
            '/camera/realsense_node/depth/color/points', 
            self.depth_callback,       
            10)
        
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status', # '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )
        
        self.retake_mask = None
        self.frame_width = None
        self.frame_height = None
        self.mask_np = np.zeros((480, 640), dtype=np.uint8)

        # State variables
        self.current_position = [0.0, 0.0, 0.0]  # Current position in NED
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.error = [0.0, 0.0, 0.0]
        self.current_mode = None  # Current flight mode
        self.armed = False  # Armed state
        self.offboard_mode = "VELOCITY"
        self.state = "ARMING"  # State machine state
        self.takeoff_altitude = -8.0  # Takeoff altitude in NED (8 meters up)
        
        # Timing variables
        self.state_change_time = self.get_clock().now()
        self.current_yaw = 0.0  # Instantaneous target yaw
        self.running = True
        self.collect_depth = False

        # Movement control variables
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.drop_odometry_x = None
        self.drop_odometry_y = None
        self.drop_odometry_z = None
        self.maintain_z = False  # Flag to indicate whether to maintain z position
        self.moving_in_z = False  # Flag to indicate explicit z movement
        self.target_acquired = False
        self.last_detection_time = None  # Track when an object was last detected
        self.initial_yaw = None  # Will store the yaw at takeoff completion

        # Timer to control the state machine
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        self.publish_offboard_timer = self.create_timer(0.01, self.publish_offboard_control_mode)  # 100Hz
        self.last_movement_time = None

        # Initialize descent tracking variables
        self.begin_descend = False
        self.descent_start_time = None  # Will be set when descent begins
        self.descent_data_saved = False

        # Teraranger LIDAR Subscription
        # Create subscriber
        self.create_subscription(
            Float32MultiArray,
            '/teraranger_evo/distances',
            self.teraranger_callback,
            10
        )
        
        # Define sensor positions [front, right, back, left]
        self.directions = ['front', 'right', 'back', 'left']
        self.processed_distances = [float('inf'), float('inf'), float('inf'), float('inf')]
        
        # Max distance when sensor returns inf (in meters)
        self.lidar_max_distance = 60.0

        # Obstacle avoidance parameters
        self.safe_distance = 5.0  # Distance threshold in meters to start slowing down
        self.min_distance = 2.0   # Minimum distance for complete stop
        
        # PID controller parameters for velocity control
        self.kp = 0.3  # Proportional gain
        self.ki = 0.05 # Integral gain
        self.kd = 0.1  # Derivative gain
        
        # PID controller state variables for each direction
        self.prev_errors = {'front': 0.0, 'right': 0.0, 'back': 0.0, 'left': 0.0}
        self.integral_terms = {'front': 0.0, 'right': 0.0, 'back': 0.0, 'left': 0.0}
        self.last_time = {'front': None, 'right': None, 'back': None, 'left': None}
        
        # Data logging variables
        self.current_velocity = [0.0, 0.0, 0.0]  # [vx, vy, vz]
        self.data_log = []
        self.log_timer = self.create_timer(0.1, self.log_data_callback)  # 10Hz data logging
        
        # Create directory for data logs if it doesn't exist
        self.log_dir = os.path.expanduser('~/drone_data_logs')
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Create a timestamp for the log file
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = os.path.join(self.log_dir, f'drone_data_{timestamp}.xlsx')
    
    def log_data_callback(self):
        """Collect data at regular intervals"""
        current_time = time.time()
        
        # Create data row
        data_row = {
            'timestamp': current_time,
            'state': self.state,
            'position_x': self.current_position[0],
            'position_y': self.current_position[1],
            'position_z': self.current_position[2],
            'velocity_x': self.current_velocity[0],
            'velocity_y': self.current_velocity[1],
            'velocity_z': self.current_velocity[2],
            'yaw': self.current_yaw,
            'distance_front': self.processed_distances[0] if len(self.processed_distances) > 0 else float('inf'),
            'distance_right': self.processed_distances[1] if len(self.processed_distances) > 1 else float('inf'),
            'distance_back': self.processed_distances[2] if len(self.processed_distances) > 2 else float('inf'),
            'distance_left': self.processed_distances[3] if len(self.processed_distances) > 3 else float('inf'),
            'error_x': self.error[0],
            'error_y': self.error[1],
            'error_z': self.error[2],
        }
        
        # Append to the data log
        self.data_log.append(data_row)

        # Periodically print the size of the data log
        if len(self.data_log) % 10 == 0:  # Every 10 entries
            self.get_logger().info(f"Data log size: {len(self.data_log)} entries")

    def save_data_to_excel(self):
        """Save collected data to Excel file"""
        self.get_logger().info(f"Attempting to save {len(self.data_log)} data entries")
    
        """Save collected data to Excel file"""
        if len(self.data_log) == 0:
            self.get_logger().warn("No data to save")
            return
            
        try:
            # Convert to DataFrame
            df = pd.DataFrame(self.data_log)
            
            # Add human-readable timestamps
            start_time = df['timestamp'].iloc[0]
            df['elapsed_time'] = df['timestamp'] - start_time
            df['datetime'] = pd.to_datetime(df['timestamp'], unit='s')
            
            # Convert radians to degrees for better readability
            df['yaw_degrees'] = df['yaw'].apply(lambda x: math.degrees(x))
            
            # Reorder columns for better readability
            columns_order = [
                'datetime', 'elapsed_time', 'state',
                'position_x', 'position_y', 'position_z',
                'velocity_x', 'velocity_y', 'velocity_z',
                'yaw', 'yaw_degrees',
                'distance_front', 'distance_right', 'distance_back', 'distance_left',
                'error_x', 'error_y', 'error_z'
            ]
            df = df[columns_order]
            
            # Save to Excel
            df.to_excel(self.log_file, index=False, engine='openpyxl')
            self.get_logger().info(f"Data saved to {self.log_file}")
        except Exception as e:
            self.get_logger().error(f"Error saving data to Excel: {e}")

    def teraranger_callback(self, msg):
            self.processed_distances = []
            # Process each distance value            
            for i, distance in enumerate(msg.data):
                direction = self.directions[i]
                
                # Handle special cases
                if distance == float('-inf'):
                    # Object too close - set to 0
                    self.processed_distances.append(0.0)
                    self.get_logger().debug(f"{direction}: Below minimum range (-inf), setting to 0.0m")
                elif distance == float('inf'):
                    # Object too far - set to max distance
                    self.processed_distances.append(self.lidar_max_distance)
                    self.get_logger().debug(f"{direction}: Above maximum range (inf), setting to {self.lidar_max_distance}m")
                elif distance != distance:  # Check for NaN
                    # Invalid reading - set to 0
                    self.processed_distances.append(self.lidar_max_distance)
                    self.get_logger().debug(f"{direction}: Invalid reading (nan), setting to 0.0m")
                else:
                    # Normal reading
                    self.processed_distances.append(distance)
                    self.get_logger().debug(f"{direction}: Valid reading: {distance:.3f}m")
            
            # Log the processed distances
            self.get_logger().info(
                f"Processed distances - Front: {self.processed_distances[0]:.2f}m, "
                f"Right: {self.processed_distances[1]:.2f}m, "
                f"Back: {self.processed_distances[2]:.2f}m, "
                f"Left: {self.processed_distances[3]:.2f}m"
            )
    
    def get_distance_in_direction(self, direction):
        """Get the distance from the LIDAR in a specific direction."""
        direction_index = {
            "forward": 0,  # front
            "right": 1,    # right
            "backward": 2, # back
            "left": 3      # left
        }
        
        idx = direction_index.get(direction, 0)
        return self.processed_distances[idx] if len(self.processed_distances) > idx else self.lidar_max_distance
    
    def calculate_velocity_with_obstacle_avoidance(self, direction, desired_speed=0.5):
        """
        Calculate velocity components with obstacle avoidance based on LIDAR readings.
        Returns velocity scaled by PID controller for collision avoidance.
        """
        # Get raw velocity components based on direction
        vx, vy = self.calculate_velocity(direction, speed=desired_speed)
        
        # Get the distance in the current movement direction
        distance = self.get_distance_in_direction(direction)
        
        # Calculate velocity scale factor using PID controller
        scale_factor = self.calculate_velocity_scale(direction, distance)
        
        # Apply scale factor to velocity components
        adjusted_vx = vx * scale_factor
        adjusted_vy = vy * scale_factor
        
        self.get_logger().info(
            f"Direction: {direction}, Distance: {distance:.2f}m, "
            f"Scale: {scale_factor:.2f}, "
            f"Velocity adjusted from ({vx:.2f}, {vy:.2f}) to ({adjusted_vx:.2f}, {adjusted_vy:.2f})"
        )
        
        return adjusted_vx, adjusted_vy

    def calculate_velocity_scale(self, direction, distance):
        """
        Calculate velocity scale factor using PID controller based on distance to obstacle.
        Returns a value between 0.0 (stop) and 0.5 (full speed).
        """
        current_time = time.time()
        
        # If distance is above safe_distance, move at full speed
        if distance >= self.safe_distance:
            # Reset PID terms when we're in safe zone
            self.prev_errors[direction] = 0.0
            self.integral_terms[direction] = 0.0
            self.last_time[direction] = current_time
            return 0.5
            
        # If distance is below min_distance, stop
        if distance <= self.min_distance:
            return 0.0
            
        # Calculate error: how far we are from safe_distance
        # As we get closer to min_distance, error grows
        error = (distance - self.min_distance) / (self.safe_distance - self.min_distance)
        
        # Initialize time if first run
        if self.last_time[direction] is None:
            self.last_time[direction] = current_time
            self.prev_errors[direction] = error
            return error  # Return proportional term only on first run
            
        # Calculate time delta
        dt = current_time - self.last_time[direction]
        if dt <= 0:
            dt = 0.05  # Use default if time hasn't advanced
            
        # Calculate PID terms
        # Proportional term
        p_term = error
        
        # Integral term with anti-windup
        self.integral_terms[direction] += error * dt
        self.integral_terms[direction] = max(0.0, min(0.5, self.integral_terms[direction]))  # Clamp
        i_term = self.ki * self.integral_terms[direction]
        
        # Derivative term
        d_term = self.kd * (error - self.prev_errors[direction]) / dt if dt > 0 else 0.0
        
        # Calculate PID output
        pid_output = self.kp * p_term + i_term + d_term
        
        # Clamp output between 0 and 1
        velocity_scale = max(0.0, min(0.5, pid_output))
        
        # Store values for next iteration
        self.prev_errors[direction] = error
        self.last_time[direction] = current_time
        
        return velocity_scale
        
    def color_callback(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            results = self.model(img)

            if not self.frame_height and not self.frame_width:
                self.frame_width = img.shape[1]
                self.frame_height = img.shape[0]
                frame_width = img.shape[1]
                frame_height = img.shape[0]
            else:
                frame_width = img.shape[1]
                frame_height = img.shape[0]
                
            center_region_width = 500  # 300 px width threshold
            center_region_height = 300  # 100 px height threshold

            # Adjust vertical position by moving the box down
            vertical_offset = 150  # Number of pixels to move down

            # Calculate total frame area
            total_frame_area = frame_width * frame_height

            inference_msg = Yolov8Inference()
            inference_msg.header.stamp = self.get_clock().now().to_msg()
            inference_msg.header.frame_id = "inference"

            # Define center bounding box region
            central_x_min = (frame_width - center_region_width) // 2
            central_x_max = (frame_width + center_region_width) // 2
            central_y_min = (frame_height - center_region_height) // 2 #+ vertical_offset
            central_y_max = (frame_height + center_region_height) // 2 #+ vertical_offset

            # Draw center bounding box (red rectangle)
            cv2.rectangle(img, (central_x_min, central_y_min), (central_x_max, central_y_max), (0, 0, 255), 2)

            # Variables to track the lowest-right object
            selected_mask = None
            selected_bbox = None
            max_position_sum = float("-inf")
            selected_bbox_area = 0

            for r in results:
                masks = getattr(r, 'masks', None)
                confidences = r.boxes.conf
                boxes = r.boxes.xyxy  # Bounding boxes (xyxy format)
                # if masks is not None:
                #     for mask in masks.data:
                #         # Debug print/visualize
                #         print("\n[DEBUG] Mask detected!")
                #         print(f"Raw mask shape: {mask.shape}")
                        
                #         # Convert and check
                #         mask_np = mask.cpu().numpy().squeeze()
                #         print(f"NumPy shape: {mask_np.shape}")
                        
                #         # Visualize (optional)
                #         cv2.imshow("YOLOv8 Mask", (mask_np * 255).astype(np.uint8))
                #         cv2.waitKey(1)
                            
                if masks is not None and boxes is not None:
                    for mask, confidence, box in zip(masks.data, confidences, boxes):
                        if confidence < 0.75:
                            continue
                        
                        x1, y1, x2, y2 = map(int, box.tolist())
                        bbox_area = (x2 - x1) * (y2 - y1)
                        position_sum = x2 + y2

                        if position_sum > max_position_sum:
                            max_position_sum = position_sum
                            selected_bbox = (x1, y1, x2, y2)
                            selected_bbox_area = bbox_area
                            selected_mask = (mask > 0.5).cpu().numpy().astype('uint8')
                            if self.retake_mask:
                                self.mask_np = selected_mask
                else:
                    print("No valid detections in this frame")
            
            if selected_bbox and selected_mask is not None and selected_mask.any() and self.state == "ALLIGN":
                x_min, y_min, x_max, y_max = selected_bbox
                self.target_acquired = True
                self.last_detection_time = self.get_clock().now()

                # Compute the center point of the selected object
                bbox_center_x = (x_min + x_max) // 2
                bbox_center_y = (y_min + y_max) // 2

                self.mask_np = selected_mask

                # Calculate area ratio
                bbox_area_ratio = (selected_bbox_area / total_frame_area) * 100

                self.get_logger().debug(f"Bounding Box Area Ratio: {bbox_area_ratio:.2f}%")
                self.get_logger().debug(f"Selected Bounding Box: ({x_min}, {y_min}) to ({x_max}, {y_max})")
                self.get_logger().debug(f"Selected Center Point: ({bbox_center_x}, {bbox_center_y})")

                # Draw the selected bounding box (green)
                cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # Draw the center point (red circle)
                cv2.circle(img, (bbox_center_x, bbox_center_y), 5, (0, 0, 255), -1)

                # Reset velocities
                self.vx = 0.0
                self.vy = 0.0
                self.vz = 0.0
                h_vx = 0.0
                h_vy = 0.0
                b_vx = 0.0
                b_vy = 0.0

                # Track time when object is centered in bounding box
                current_time = self.get_clock().now()

                # Check if the object center is within the central bounding box
                is_centered = (central_x_min <= bbox_center_x <= central_x_max and 
                            central_y_min <= bbox_center_y <= central_y_max)

                if is_centered:
                    # Start tracking time when object first becomes centered
                    if not hasattr(self, '_centered_start_time'):
                        self._centered_start_time = current_time
                        self.get_logger().debug("Object entered center region, starting timer")
                    
                    # Check if object has been centered for more than 5 seconds
                    centered_duration = (current_time - self._centered_start_time).nanoseconds / 1e9
                    self.get_logger().debug(f"Object centered for {centered_duration:.2f} seconds")
                    
                    if centered_duration > 5.0:
                        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                        self.get_logger().info("Object centered for 5+ seconds. Transitioning to HOVER.")
                        self.change_state("HOVER")
                        return
                        
                else:
                    # Reset centered time if object moves out of center
                    if hasattr(self, '_centered_start_time'):
                        self.get_logger().debug("Object left center region, resetting timer")
                        delattr(self, '_centered_start_time')

                # Proportional control for vertical movement
                horizontal_error = 0.0
                vertical_error = (bbox_center_y - ((central_y_min + central_y_max) // 2))
                vertical_p_gain = 0.005  # Adjust this for smoother vertical movement
                self.vz = vertical_error * vertical_p_gain

                # Clip vertical velocity
                self.vz = max(-0.5, min(0.5, self.vz))

                # Horizontal movement with proportional control and velocity calculation
                if bbox_center_x < central_x_min:
                    # Move left with proportional speed
                    horizontal_error = central_x_min - bbox_center_x
                    horizontal_p_gain = 0.005  # Adjust for smoother horizontal movement
                    speed = min(0.5, horizontal_error * horizontal_p_gain)
                    h_vx, h_vy = self.calculate_velocity_with_obstacle_avoidance("left", desired_speed=speed)
                    # self.vx, self.vy = vx, vy
                elif bbox_center_x > central_x_max:
                    # Move right with proportional speed
                    horizontal_error = bbox_center_x - central_x_max
                    horizontal_p_gain = 0.005  # Adjust for smoother horizontal movement
                    speed = min(0.5, horizontal_error * horizontal_p_gain)
                    h_vx, h_vy = self.calculate_velocity_with_obstacle_avoidance("right", desired_speed=speed)
                    # self.vx, self.vy = vx, vy

                # Check if area is too large - move backward
                if bbox_area_ratio > 80:
                    b_vx, b_vy = self.calculate_velocity_with_obstacle_avoidance("backward", desired_speed=0.25)
                    # self.vx, self.vy = vx, vy
                
                # if (h_vx and h_vy and b_vx and b_vy) != 0:
                # Calculate resultant velocity using vector addition
                self.vx = h_vx + b_vx
                self.vy = h_vy + b_vy

                # Optional: Normalize velocity if total speed exceeds 0.5
                total_speed = np.sqrt(self.vx**2 + self.vy**2)
                if total_speed > 0.5:
                    self.vx /= total_speed
                    self.vy /= total_speed
                
                self.error = [horizontal_error, vertical_error, 0.0]
                self.publish_trajectory_setpoint(vx=self.vx, vy=self.vy, vz=self.vz)
            
            # Image publishing code
            annotated_frame = results[0].plot()
            final_image = cv2.addWeighted(annotated_frame, 0.8, img, 0.2, 0)

            img_msg = self.bridge.cv2_to_imgmsg(final_image, encoding="bgr8")
            self.img_pub.publish(img_msg)
            self.inference_pub.publish(inference_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def calculate_drop_point_position(self, cx, cy, cz, drop_X, drop_Y, drop_Z):
        """
        Calculate the drop point position in NED coordinates (odometry frame)
        based on the camera's 3D point cloud measurements.
        
        Args:
        cx (float): Current x position in NED frame (North)
        cy (float): Current y position in NED frame (East)
        cz (float): Current z position in NED frame (Down)
        drop_X (float): X coordinate of drop point in camera frame (right)
        drop_Y (float): Y coordinate of drop point in camera frame (down)
        drop_Z (float): Z coordinate of drop point in camera frame (forward)
        
        Returns:
        tuple: Final x, y, z coordinates in NED frame and distance
        """
        # Yaw angle for rotation
        yaw = self.current_yaw  # Current yaw angle in radians

        # Camera frame to NED transformation
        # Assuming camera is mounted facing forward on the drone:
        # Camera X (right) -> Drone Y (East) when yaw=0
        # Camera Y (down) -> Drone Z (Down)
        # Camera Z (forward) -> Drone X (North) when yaw=0
        
        # First, convert from camera frame to drone body frame
        # This accounts for how the camera is physically mounted on the drone
        body_frame_X = drop_Z  # Camera Z (forward) -> Body X (forward)
        body_frame_Y = drop_X  # Camera X (right) -> Body Y (right)
        body_frame_Z = drop_Y  # Camera Y (down) -> Body Z (down)
        
        camera_offset = np.array([0.10, 0.0, 0.0])

        # Now rotate the body frame point based on drone's current yaw to get to NED frame
        # Rotation matrix for yaw (rotation around z-axis)
        rotation_matrix = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        body_frame_point = np.array([body_frame_X, body_frame_Y, body_frame_Z])
        body_frame_point = body_frame_point + camera_offset
        rotated_point = rotation_matrix @ body_frame_point
        
        # Calculate final position by adding current position and rotated point
        final_x = cx + rotated_point[0]  # North
        final_y = cy + rotated_point[1]  # East
        final_z = cz + rotated_point[2]  # Down
        
        # Calculate distance from camera
        distance = np.linalg.norm(np.array([drop_X, drop_Y, drop_Z]))
        
        # Log the movement details
        self.get_logger().info(
            f"Drop Point Calculation: "
            f"Initial Position ({cx:.2f}, {cy:.2f}, {cz:.2f}) | "
            f"Yaw: {math.degrees(yaw):.1f}° | "
            f"Camera Frame Point: ({drop_X:.2f}, {drop_Y:.2f}, {drop_Z:.2f}) | "
            f"Body Frame Point: ({body_frame_X:.2f}, {body_frame_Y:.2f}, {body_frame_Z:.2f}) | "
            f"Distance from Camera: {distance:.2f}m | "
            f"Final NED Position: ({final_x:.2f}, {final_y:.2f}, {final_z:.2f})"
        )
        
        return final_x, final_y, final_z, distance

    def publish_drop_point(self, x, y, z):
        """
        Publish drop point coordinates in odometry frame
        
        Args:
        x (float): X coordinate in odometry frame
        y (float): Y coordinate in odometry frame
        z (float): Z coordinate in odometry frame
        """
        # Create Point message
        point_msg = Point()
        point_msg.x = x
        point_msg.y = y
        point_msg.z = z
        
        # Publish the point
        self.drop_point_pub.publish(point_msg)

    def depth_callback(self, msg):
        # Check if the point cloud is organized or unorganized
        is_organized = msg.height > 1
        self.get_logger().info(f"Point cloud is {'organized' if is_organized else 'unorganized'}: width={msg.width}, height={msg.height}")
        
        # Get all points including NaN values
        depth_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
        self.get_logger().info(f"Total points: {len(depth_points)}")
        
        if self.state == "HOVER" and self.target_acquired:
            image_width = self.frame_width  # 1280
            image_height = self.frame_height  # 720

            # Camera intrinsics
            fx = 906.370849609375 # fx = 554.2547
            fy = 905.2732543945312 # fy = 554.2547  
            cx = 659.2393798828125 # cx = 320.5  
            cy = 368.3586120605469 # cy = 240.5

            # For unorganized point clouds, we need to project the points to a grid
            if not is_organized:
                # Define expected image dimensions
                
                # Create an empty depth image
                depth_image = np.full((image_height, image_width), np.nan)
                
                # Project 3D points to 2D grid
                for point in depth_points:
                    x, y, z = point
                    
                    # Skip invalid points
                    if math.isnan(x) or math.isnan(y) or math.isnan(z) or z <= 0:
                        continue
                    
                    # Project 3D point to 2D pixel coordinates
                    u = int((x * fx / z) + cx)
                    v = int((y * fy / z) + cy)
                    
                    # Check if point is within image bounds
                    if 0 <= u < image_width and 0 <= v < image_height:
                        # Store the depth value (z) in the image
                        # If multiple points project to the same pixel, keep the closest one
                        if np.isnan(depth_image[v, u]) or z < depth_image[v, u]:
                            depth_image[v, u] = z
            else:
                # For organized point clouds, reshape directly
                depth_data = np.array([point[2] if not math.isnan(point[2]) else np.nan for point in depth_points])
                depth_image = depth_data.reshape((msg.height, msg.width))
                
                # Need to resize if dimensions don't match expectations
                if msg.height != self.frame_height or msg.width != self.frame_width:
                    depth_image = cv2.resize(depth_image, (self.frame_width, self.frame_height))
            
            # Check if we have valid depth data
            valid_depth_count = np.sum(~np.isnan(depth_image))
            self.get_logger().info(f"Valid depth points after projection: {valid_depth_count}")
            
            if valid_depth_count > 0:
                # Normalize the depth image for visualization
                min_depth = np.nanmin(depth_image)
                max_depth = np.nanmax(depth_image)
                depth_image_normalized = (depth_image - min_depth) / (max_depth - min_depth)
                depth_image_normalized[np.isnan(depth_image_normalized)] = 0  # Set NaN to 0 for visualization
                
                # Resize mask to match depth image dimensions
                mask_resized = cv2.resize(self.mask_np, (self.frame_width, self.frame_height))
                
                # Apply the mask - Keep only solar panel depth values
                fused_depth_image = np.where(mask_resized > 0, depth_image, np.nan)

                # Calculate white pixel area for fused depth image
                white_area_fused = np.sum(~np.isnan(fused_depth_image))
                print(f"White pixel area in fused depth image: {white_area_fused}")

                # Normalize masked depth image for visualization
                min_fused = np.nanmin(fused_depth_image)
                max_fused = np.nanmax(fused_depth_image)
                fused_depth_image_normalized = (fused_depth_image - min_fused) / (max_fused - min_fused)

                # Calculate white pixel area for normalized fused depth image
                white_area_normalized = np.sum(~np.isnan(fused_depth_image_normalized))
                print(f"White pixel area in normalized fused depth image: {white_area_normalized}")
                
                # Count the number of valid points in the fused image
                valid_fused_count = np.sum(~np.isnan(fused_depth_image))
                self.get_logger().info(f"Valid points after masking: {valid_fused_count}")
                
                if valid_fused_count > 0:
                
                    ## Extract 3D coordinates from the fused depth data
                    points_3d = []
                    pixel_coords = []
                    for v in range(image_height):
                        for u in range(image_width):
                            if mask_resized[v, u] > 0:  # Only use masked pixels
                                Z = fused_depth_image[v, u]  # Depth value
                                if np.isnan(Z):  # Skip NaN values
                                    continue
                                X = (u - cx) * Z / fx
                                Y = (v - cy) * Z / fy
                                points_3d.append([X, Y, Z])
                                pixel_coords.append((u, v))  # Store corresponding image coordinates

                    points_3d = np.array(points_3d)
                    
                    if points_3d.shape[0] > 3:  # Ensure enough points for plane fitting
                        # Convert to Open3D point cloud
                        pcd = o3d.geometry.PointCloud()
                        pcd.points = o3d.utility.Vector3dVector(points_3d)

                        # Fit a plane using RANSAC
                        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                                ransac_n=3,
                                                                num_iterations=1000)
                        a, b, c, d = plane_model  # Plane equation: ax + by + cz + d = 0

                        # Compute the tilt angle (angle between plane normal and Z-axis)
                        tilt_angle_rad = np.arccos(abs(c) / np.sqrt(a**2 + b**2 + c**2))

                        if tilt_angle_rad > (math.pi/4):
                            tilt_angle_rad = (math.pi/2) - tilt_angle_rad
                            
                        tilt_angle_deg = np.degrees(tilt_angle_rad)

                        angle_msg = Float64(data=float(tilt_angle_rad))

                        # Calculate the centroid of all valid points (middle of the depth image)
                        if len(points_3d) > 0:
                            centroid = np.nanmean(points_3d, axis=0)
                            drop_X = centroid[0]  # X coordinate of centroid
                            drop_Y = centroid[1]  # Y coordinate of centroid
                            drop_Z = centroid[2]  # Z coordinate of centroid
                            
                            self.get_logger().info(f"Drop point at centroid: X={drop_X:.2f}, Y={drop_Y:.2f}, Z={drop_Z:.2f}")
                        
                        # Extract inlier and outlier points
                        inlier_points = np.asarray(pcd.points)[inliers]
                        outlier_points = np.asarray(pcd.points)[[i for i in range(len(pcd.points)) if i not in inliers]]
                        
                        # Visualize using matplotlib - alternative approach without plot_surface
                        # try:
                        #     # Create a new figure
                        #     fig = plt.figure(figsize=(10, 8))
                        #     ax = fig.add_subplot(111, projection='3d')
                            
                        #     # Plot inlier points in red with X and Y swapped
                        #     if len(inlier_points) > 0:
                        #         ax.scatter(inlier_points[:, 1], inlier_points[:, 0], inlier_points[:, 2], 
                        #                 color='red', s=5, label='Plane Inliers')
                            
                        #     # Plot outlier points in blue with X and Y swapped
                        #     if len(outlier_points) > 0:
                        #         ax.scatter(outlier_points[:, 1], outlier_points[:, 0], outlier_points[:, 2], 
                        #                 color='blue', s=5, label='Outliers')
                            
                        #     # Plot the centroid/drop point as a large green dot with X and Y swapped
                        #     ax.scatter([drop_Y], [drop_X], [drop_Z], color='green', s=100, label='Drop Point')
                            
                        #     # Create the plane visualization - with X and Y swapped
                        #     y_min, y_max = np.min(points_3d[:, 0]), np.max(points_3d[:, 0])  # Note the swap here
                        #     x_min, x_max = np.min(points_3d[:, 1]), np.max(points_3d[:, 1])  # Note the swap here
                            
                        #     # Create four corners of a rectangle in the plane with X and Y swapped
                        #     yy, xx = np.meshgrid([y_min, y_max], [x_min, x_max])  # Note the swap here
                            
                        #     # For the plane equation ax + by + cz + d = 0
                        #     # When we swap x and y, we need to use the equation bx + ay + cz + d = 0
                        #     z_corners = (-d - b*xx - a*yy) / c if c != 0 else np.zeros_like(xx)  # Note a and b swapped
                            
                        #     # Plot the plane as a wireframe with X and Y swapped
                        #     ax.plot_wireframe(xx, yy, z_corners, color='yellow', alpha=0.5)
                            
                        #     # Add normal vector from centroid with X and Y swapped
                        #     normal = np.array([b, a, c])  # Note a and b swapped
                        #     normal = normal / np.linalg.norm(normal) * 0.1  # Scale for visualization
                        #     ax.quiver(drop_Y, drop_X, drop_Z, normal[0], normal[1], normal[2], 
                        #             color='green', length=0.2, normalize=True)
                            
                        #     # Set labels to reflect the swap
                        #     ax.set_xlabel('Y')  # This is now showing on the X axis
                        #     ax.set_ylabel('X')  # This is now showing on the Y axis
                        #     ax.set_zlabel('Z')
                        #     ax.set_title(f'RANSAC Plane Fit (X-Y Swapped): {b:.2f}x + {a:.2f}y + {c:.2f}z + {d:.2f} = 0')
                            
                        #     # Add an annotation with the tilt angle in the top left corner
                        #     ax.annotate(f'Tilt angle: {tilt_angle_deg:.2f}°', 
                        #                 xy=(0.02, 0.95), 
                        #                 xycoords='axes fraction',
                        #                 fontsize=12, 
                        #                 bbox=dict(boxstyle="round,pad=0.3", fc="white", ec="gray", alpha=0.8))

                        #     # Add a legend
                        #     ax.legend()
                            
                        #     # Set equal aspect ratio
                        #     x_range = x_max - x_min
                        #     y_range = y_max - y_min
                        #     z_min, z_max = np.min(points_3d[:, 2]), np.max(points_3d[:, 2])
                        #     z_range = z_max - z_min
                        #     max_range = max(x_range, y_range, z_range)
                        #     mid_x = (x_max + x_min) / 2
                        #     mid_y = (y_max + y_min) / 2
                        #     mid_z = (z_max + z_min) / 2
                        #     ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
                        #     ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
                        #     ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)
                            
                        #     # Save the figure
                        #     timestamp = self.get_clock().now().to_msg().sec
                        #     plt.savefig(f"ransac_plane_{timestamp}.png")
                        #     plt.close() 
                            
                        #     # Display the plot without blocking
                        #     # plt.show()
                        #     # plt.pause(0.5)
                        
                        # except Exception as e:
                        #     self.get_logger().error(f"Error creating matplotlib visualization: {e}")

                        # Calculate drop point position in odometry frame
                        # if hasattr(self, 'current_x') and hasattr(self, 'current_y') and hasattr(self, 'current_z'):
                        drone_pos = self.current_position
                        current_x, current_y, current_z = drone_pos

                        self.drop_odometry_x, self.drop_odometry_y, self.drop_odometry_z, drop_distance = self.calculate_drop_point_position(
                            current_x, current_y, current_z,
                            drop_X, drop_Y, drop_Z
                        )
                        
                        diff_x = self.drop_odometry_x - current_x
                        diff_y = self.drop_odometry_y - current_y
                        diff_z = self.drop_odometry_z - current_z
                        
                        if abs(diff_x)>8 or abs(diff_y)>8:
                            self.retake_mask = True
                            return
                        else:
                            self.retake_mask = False

                        # Print all information
                        self.get_logger().info("\n--- Deployment Position Calculation ---")
                        self.get_logger().info(f"Current Drone Position (NED): X={current_x:.3f}m, Y={current_y:.3f}m, Z={current_z:.3f}m")
                        self.get_logger().info(f"Deployment Position (NED):    X={self.drop_odometry_x:.3f}m, Y={self.drop_odometry_y:.3f}m, Z={self.drop_odometry_z:.3f}m")
                        self.get_logger().info(f"Difference (Deployment - Current): ΔX={diff_x:.3f}m, ΔY={diff_y:.3f}m, ΔZ={diff_z:.3f}m")
                        self.get_logger().info("------------------------------------")

                        # Prompt for user confirmation
                        # user_confirmation = input("Do you want to proceed with publishing? (y/n/r): ").strip().lower()

                        # if user_confirmation == 'y':
                        # Proceed with publishing
                                            # Publish inclination angle
                        self.inclination_pub.publish(angle_msg)
                        self.publish_drop_point(self.drop_odometry_x, self.drop_odometry_y, self.drop_odometry_z)
                        
                        # Identify the bottom-right corner
                        bottom_right_idx = np.argmax(points_3d[:, 0] + points_3d[:, 2])  # Rightmost + lowest point
                        X_br, Y_br, Z_br = points_3d[bottom_right_idx]

                        # Compute the dropping point in 3D (move 8cm up & left)
                        drop_X = X_br - 54  # Move 8cm left
                        drop_Y = Y_br + 54  # Move 8cm up
                        drop_Z = Z_br  # Same depth

                        # Project to 2D using intrinsic matrix
                        drop_u = int((drop_X * fx / drop_Z) + cx)
                        drop_v = int((drop_Y * fy / drop_Z) + cy)

                        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                        # Plot full depth intensity map
                        plt.figure(figsize=(10, 6))
                        plt.imshow(depth_image_normalized, cmap='jet', interpolation='nearest')
                        plt.colorbar(label='Depth Intensity')
                        plt.scatter(drop_u, drop_v, color='red', s=100, label='Dropping Point')
                        plt.text(10, 30, f"Slant Angle: {tilt_angle_deg:.2f}°", fontsize=12, color='white', bbox=dict(facecolor='black', alpha=0.5))
                        plt.legend()
                        plt.title("Full Depth Intensity Map with Dropping Point")
                        # plt.show()
                        plt.savefig(f'full_depth_map_{timestamp}.png', bbox_inches='tight', dpi=300)
                        plt.close()  # Close the figure to free memory

                        # Plot fused depth intensity map (solar panel only)
                        plt.figure(figsize=(10, 6))
                        panel_only = np.copy(fused_depth_image_normalized)
                        panel_only[np.isnan(panel_only)] = 0  # Set background to 0 for better visibility

                        plt.imshow(panel_only, cmap='gray', interpolation='nearest')
                        plt.colorbar(label='Depth Intensity (Solar Panel Only)')
                        plt.scatter(drop_u, drop_v, color='red', s=100, label='Dropping Point')
                        plt.text(10, 30, f"Slant Angle: {tilt_angle_deg:.2f}°", fontsize=12, color='white', bbox=dict(facecolor='black', alpha=0.5))
                        plt.legend()
                        plt.title("Isolated Solar Panel Depth Map with Dropping Point")
                        # plt.show()
                        plt.savefig(f'panel_depth_map_{timestamp}.png', bbox_inches='tight', dpi=300)
                        plt.close() 

                        # self.save_data_to_excel()
                        # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                        self.change_state("DEPLOY_POS")
                else:
                    self.get_logger().warn("No valid fused depth points - mask may not be aligned with depth data")
            else:
                self.get_logger().warn("No valid depth points after projection")

                    # elif user_confirmation == 'n':
                    #     print("Process cancelled by user.")
                    #     self.change_state("RETURN_TO_LAUNCH")
                    # elif user_confirmation == 'r':
                    #     self.retake_mask = True
                    # else:
                    #     print("Invalid input by user.")
                    #     self.change_state("RETURN_TO_LAUNCH")

    def calculate_velocity(self, direction, speed=0.5):
        """
        Calculate velocity components based on initial yaw after takeoff.
        
        Args:
            direction (str): Movement direction ('forward', 'backward', 'right', 'left')
            speed (float): Speed multiplier between 0 and 0.5
        
        Returns:
            tuple: Scaled vx and vy velocities
        """
        # Ensure speed is between 0 and 0.5
        speed = max(0.0, min(0.5, speed))
        
        # Use initial_yaw if set, otherwise use current_yaw (shouldn't happen)
        yaw = self.current_yaw #self.initial_yaw if hasattr(self, 'initial_yaw') and self.initial_yaw is not None else self.current_yaw
        
        # Base direction vectors (at 0° yaw)
        base_vectors = {
            "forward": (1, 0),
            "backward": (-1, 0),
            "right": (0, 1),
            "left": (0, -1)
        }
        
        # Get the base vector
        base_vx, base_vy = base_vectors.get(direction, (0, 0))
        
        # Apply rotation
        vx = base_vx * math.cos(yaw) - base_vy * math.sin(yaw)
        vy = base_vx * math.sin(yaw) + base_vy * math.cos(yaw)
        
        # Find max absolute value to scale
        max_val = max(abs(vx), abs(vy))
        
        # Scale to [-0.5, 0.5] range if needed
        if max_val > 0.5:
            vx /= max_val
            vy /= max_val
        
        # Apply speed multiplier
        vx *= speed
        vy *= speed
        
        self.get_logger().info(
            f"Moving {direction} relative to initial yaw: {(yaw):.1f} rad, {math.degrees(yaw):.1f}° | "
            f"Speed: {speed:.2f} | Velocities - X: {vx:.2f}, Y: {vy:.2f}"
        )
        
        return vx, vy

    def odometry_callback(self, msg):
        """Callback to update the current position."""
        self.current_position = [
            msg.position[0],  # X in NED
            msg.position[1],  # Y in NED
            msg.position[2],  # Z in NED
        ]
        self.angular_velocity = [
            msg.angular_velocity[0],  # Roll angular velocity
            msg.angular_velocity[1],  # Pitch angular velocity
            msg.angular_velocity[2],  # Yaw angular velocity
        ]
    
        # Extract quaternion and convert to yaw
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        roll, pitch, yaw = self.quaternion_to_euler(q)
        self.current_yaw = yaw

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def vehicle_status_callback(self, msg):
        """Callback to update the current mode."""
        self.current_mode = msg.nav_state
        if msg.arming_state == 2:
            self.armed = True
        else:
            self.armed = False

    def publish_offboard_control_mode(self):
        """Publish OffboardControlMode message."""
        try:
            if self.offboard_mode == "VELOCITY":
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                offboard_msg.position = False
                offboard_msg.velocity = True  # Enable velocity control
                offboard_msg.acceleration = False
                offboard_msg.attitude = False
                offboard_msg.body_rate = False
                self.offboard_control_mode_publisher.publish(offboard_msg)
            elif self.offboard_mode == "POSITION":
                offboard_msg = OffboardControlMode()
                offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                offboard_msg.position = True
                offboard_msg.velocity = False  # Enable velocity control
                offboard_msg.acceleration = False
                offboard_msg.attitude = False
                offboard_msg.body_rate = False
                self.offboard_control_mode_publisher.publish(offboard_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing offboard control mode: {str(e)}")

    def publish_trajectory_setpoint(self, vx=0.0, vy=0.0, vz=0.0, yaw_rate=0.0):
        """Publish a trajectory setpoint in velocity mode."""
        try:
            # Store current velocity for logging
            self.current_velocity = [vx, vy, vz]
            if self.offboard_mode == "VELOCITY":
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = vx
                trajectory_msg.velocity[1] = vy
                trajectory_msg.velocity[2] = vz
                trajectory_msg.position[0] = float('nan')
                trajectory_msg.position[1] = float('nan')
                trajectory_msg.position[2] = float('nan')
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                self.trajectory_setpoint_publisher.publish(trajectory_msg)
            elif self.offboard_mode == "POSITION":
                trajectory_msg = TrajectorySetpoint()
                trajectory_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
                trajectory_msg.velocity[0] = float('nan')
                trajectory_msg.velocity[1] = float('nan')
                trajectory_msg.velocity[2] = float('nan')
                trajectory_msg.position[0] = vx
                trajectory_msg.position[1] = vy
                trajectory_msg.position[2] = vz
                trajectory_msg.acceleration[0] = float('nan')
                trajectory_msg.acceleration[1] = float('nan')
                trajectory_msg.acceleration[2] = float('nan')
                trajectory_msg.yaw = float('nan')
                self.trajectory_setpoint_publisher.publish(trajectory_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing trajectory setpoint: {str(e)}")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        """Publish a VehicleCommand."""
        try:
            msg = VehicleCommand()
            msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            msg.param1 = param1
            msg.param2 = param2
            msg.param3 = param3
            msg.command = command
            msg.target_system = 1
            msg.target_component = 1
            msg.source_system = 1
            msg.source_component = 1
            msg.from_external = True
            self.vehicle_command_publisher.publish(msg)
            self.get_logger().info(f"Published VehicleCommand: command={command}, param1={param1}, param2={param2}, param3={param3}")
        except Exception as e:
            self.get_logger().error(f"Error publishing vehicle command: {str(e)}")

    def move_to_position(self, target_x, target_y, target_z, tolerance=0.1):
            """
            Move to a desired position using a P controller and hold position for 5 seconds.
            
            Args:
                target_x (float): Target X position in NED frame
                target_y (float): Target Y position in NED frame
                target_z (float): Target Z position in NED frame
                tolerance (float): Acceptable distance to target (in meters)
            """        

            # Check if TeraRanger detects any obstacle within safety distance
            if hasattr(self, 'processed_distances') and self.processed_distances:
                for i, distance in enumerate(self.processed_distances):
                    if distance < 5.0:
                        direction = self.directions[i] if hasattr(self, 'directions') else f"Sensor {i}"
                        self.get_logger().warning(f"Safety stop! Obstacle detected {distance:.2f}m from {direction}.")
                        self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)
                        return

            # P controller gains
            kp_x = 0.5  # Proportional gain for X
            kp_y = 0.5  # Proportional gain for Y
            kp_z = 0.5  # Proportional gain for Z
            
            # Maximum velocity limits
            max_velocity = 0.5  # m/s
            
            # Position stability timer variables
            stable_start_time = None
            required_stable_duration = 2.0  # seconds
            
            # Calculate position error
            error_x = target_x - self.current_position[0]
            error_y = target_y - self.current_position[1]
            error_z = target_z - self.current_position[2]

            self.error = [error_x, error_y, error_z]
            
            # Calculate distance to target
            distance = (error_x**2 + error_y**2 + error_z**2)**0.5
            
            # Check if we're within tolerance
            if self.begin_descend:
                # Get current time to check if 15 seconds have passed since descent began
                current_time = self.get_clock().now()
                
                # Initialize descent_start_time if not already set
                if not hasattr(self, 'descent_start_time') or self.descent_start_time is None:
                    self.descent_start_time = current_time
                    self.descent_data_saved = False
                    self.get_logger().info("Starting descent timer...")
                    
                # Calculate elapsed time since descent began
                try:
                    elapsed_seconds = (current_time - self.descent_start_time).to_msg().sec + (current_time - self.descent_start_time).to_msg().nanosec / 1e9
                    
                    # Check if 15 seconds have passed and data hasn't been saved yet
                    if elapsed_seconds >= 15.0 and (not hasattr(self, 'descent_data_saved') or not self.descent_data_saved):
                        self.get_logger().info("15 seconds elapsed since descent began. Saving data to Excel...")
                        self.save_data_to_excel()
                        self.descent_data_saved = True
                        self.change_state("LANDING")
                        
                    self.get_logger().info(f"Descent time: {elapsed_seconds:.1f} seconds")
                except TypeError as e:
                    # In case of error, reinitialize and try again next cycle
                    self.get_logger().error(f"Timer error: {e}. Resetting timer.")
                    self.descent_start_time = current_time
                    elapsed_seconds = 0.0

                self.get_logger().info("Reached target position! Maintaining X/Y while beginning descent...")
            
                # Continue using P controller for X and Y
                velocity_x = kp_x * error_x
                velocity_y = kp_y * error_y
                
                # Constant descent velocity
                velocity_z = 0.1  # constant +0.1 m/s downward (in NED, positive Z is downward)
                
                self.get_logger().info(f"Maintaining X/Y: X={self.current_position[0]:.3f}m, Y={self.current_position[1]:.3f}m")
                self.get_logger().info(f"Descending at constant rate: {velocity_z:.1f} m/s")
                
                # Send velocity commands
                self.publish_trajectory_setpoint(vx=velocity_x, vy=velocity_y, vz=velocity_z)
                
            elif (abs(error_x) < tolerance) and (abs(error_y) < tolerance) and (abs(error_z) < 0.2):
                self.begin_descend = True

            else:
                # # Reset stability timer if we drift outside tolerance
                # stable_start_time = None
                
                # Calculate velocity commands using P controller
                velocity_x = kp_x * error_x
                velocity_y = kp_y * error_y
                velocity_z = kp_z * error_z
                
                # Apply velocity limits
                magnitude = (velocity_x**2 + velocity_y**2 + velocity_z**2)**0.5
                if magnitude > max_velocity:
                    scaling_factor = max_velocity / magnitude
                    velocity_x *= scaling_factor
                    velocity_y *= scaling_factor
                    velocity_z *= scaling_factor
                
                self.get_logger().info("\n--- Deployment Position Execution ---")
                self.get_logger().info(f"Current Drone Position (NED): X={self.current_position[0]:.3f}m, Y={self.current_position[1]:.3f}m, Z={self.current_position[2]:.3f}m")
                self.get_logger().info(f"Error (Deployment - Current): ΔX={error_x:.3f}m, ΔY={error_y:.3f}m, ΔZ={error_z:.3f}m")
                self.get_logger().info(f"Velocity: X={velocity_x:.3f}m/s, Y={velocity_y:.3f}m/s, Z={velocity_z:.3f}m/s")
                self.get_logger().info("------------------------------------")
                
                # Send velocity commands
                self.publish_trajectory_setpoint(vx=velocity_x, vy=velocity_y, vz=velocity_z)

    def arm_drone(self):
        """Command the drone to arm."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def publish_takeoff(self):
        """Send takeoff command to PX4."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 2.0)  # original take off altitude
        self.get_logger().info(f"Sending takeoff command.")

    def time_since_state_change(self):
        """Get time since last state change in seconds."""
        return (self.get_clock().now() - self.state_change_time).nanoseconds / 1e9

    def change_state(self, new_state):
        """Change state with proper logging and timing."""
        self.get_logger().info(f"State transition: {self.state} -> {new_state}")
        self.state = new_state
        self.state_change_time = self.get_clock().now()

    def timer_callback(self):
        """Main loop that implements the state machine."""
        try:
            if self.state == "ARMING":
                if not self.armed:
                    if self.current_mode != 4:
                        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                    self.arm_drone()
                    self.publish_takeoff()
                else:
                    if self.current_mode != 4:
                        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                    self.get_logger().info("Drone armed and taking off")
                    
                    if self.time_since_state_change() >= 10.0:  # Wait for 15 seconds before switching to TAKEOFF
                        self.change_state("TAKEOFF")

            elif self.state == "TAKEOFF":
                if self.time_since_state_change() >= 2.0:  # Wait for 10 seconds before switching to ALLIGN
                    self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                    # self.offboard_mode = "POSITION"
                    # self.publish_trajectory_setpoint(vx=-0.485, vy=1.620, vz=(-7.554+5))
                    self.change_state("ALLIGN")
                    self.target_acquired = False
                    self.last_detection_time = None
                    # self.initial_yaw = True  # Will store the yaw at takeoff completion
                
            elif self.state == "ALLIGN":
                # If no object detected, move upwards continuously
                if not self.target_acquired or (self.last_detection_time and 
                (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9 > 1.0):
                    # No target or haven't seen one recently (more than 1 second)
                    self.vz = -0.5  # Move upwards
                    self.vx = 0.0
                    self.vy = 0.0
                    self.publish_trajectory_setpoint(vx=self.vx, vy=self.vy, vz=self.vz)
                    self.get_logger().debug("Moving upward to search for object")
                    self.last_movement_time = self.get_clock().now()  # Update last movement time
                
                # If any velocity is non-zero, we're making adjustments
                if abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.vz) > 0.01:
                    self.last_movement_time = self.get_clock().now()  # Update last movement time
                
                # Check if we've been stable (no movement) for 5 seconds
                # if self.target_acquired and ((self.get_clock().now() - self.last_movement_time).nanoseconds / 1e9 >= 2.0):
                #     self.get_logger().info("No adjustments for 2 seconds, transitioning to HOVER")
                #     self.change_state("HOVER")
                
                # Check if we've been in ALLIGN state for more than 20 seconds without finding an object
                if self.time_since_state_change() >= 7.0 and not self.target_acquired:
                    self.get_logger().info("No object detected for 20 seconds, transitioning to LANDING")
                    self.change_state("LANDING")
            
            elif self.state == "DEPLOY_POS":
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.move_to_position(target_x=self.drop_odometry_x, target_y=self.drop_odometry_y, target_z=(self.drop_odometry_z - 3.0))

            elif self.state == "HOVER":
                # self.publish_trajectory_setpoint(vx=0.0, vy=0.0, vz=0.0)  # Stay in place
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 3.0)
                # if self.time_since_state_change() >= 10.0:  # Hover for 20 seconds
                #     self.collect_depth = True
                if self.time_since_state_change() >= 50.0:  # Hover for 20 seconds
                    self.change_state("LANDING")

            elif self.state == "LANDING":
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)
                self.get_logger().info("Landing...")
                if self.time_since_state_change() >= 5.0:  # Wait for 5 seconds before declaring landed
                    self.change_state("LANDED")
            
            elif self.state == "RETURN_TO_LAUNCH":
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 5.0)
                self.get_logger().info("Returning...")

                # # Add periodic saving during hover
                # if self.time_since_state_change() >= 2.5:  # Save halfway through hover
                #     self.get_logger().info("Saving data during hover")
                #     self.save_data_to_excel()

                if self.time_since_state_change() >= 5.0:  # Wait for 5 seconds before declaring landed
                    self.change_state("LANDED")

            elif self.state == "LANDED":
                self.get_logger().info("Drone has landed successfully.")
                # Do nothing else, we're done

        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")
            # Safety fallback - try to land if something goes wrong
            if self.state != "LANDING" and self.state != "LANDED":
                self.get_logger().error("Error detected, attempting emergency landing")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 4.0, 6.0)
                self.change_state("LANDING")

def main(args=None):
    rclpy.init(args=args)
    node = ZDCalNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down...")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()