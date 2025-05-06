#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class EvoTowerSubscriber(Node):
    def __init__(self):
        super().__init__('evo_tower_subscriber')
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/teraranger_evo/distances',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        
        # Define sensor positions [front, right, back, left]
        self.directions = ['front', 'right', 'back', 'left']
        
        # Max distance when sensor returns inf (in meters)
        self.max_distance = 60.0
        
        self.get_logger().info("Evo Tower Subscriber initialized. Waiting for data...")

    def listener_callback(self, msg):
        # Process each distance value
        processed_distances = []
        
        for i, distance in enumerate(msg.data):
            direction = self.directions[i]
            
            # Handle special cases
            if distance == float('-inf'):
                # Object too close - set to 0
                processed_distances.append(0.0)
                self.get_logger().debug(f"{direction}: Below minimum range (-inf), setting to 0.0m")
            elif distance == float('inf'):
                # Object too far - set to max distance
                processed_distances.append(self.max_distance)
                self.get_logger().debug(f"{direction}: Above maximum range (inf), setting to {self.max_distance}m")
            elif distance != distance:  # Check for NaN
                # Invalid reading - set to 0
                processed_distances.append(0.0)
                self.get_logger().debug(f"{direction}: Invalid reading (nan), setting to 0.0m")
            else:
                # Normal reading
                processed_distances.append(distance)
                self.get_logger().debug(f"{direction}: Valid reading: {distance:.3f}m")
        
        # Log the processed distances
        self.get_logger().info(
            f"Processed distances - Front: {processed_distances[0]:.2f}m, "
            f"Right: {processed_distances[1]:.2f}m, "
            f"Back: {processed_distances[2]:.2f}m, "
            f"Left: {processed_distances[3]:.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    
    subscriber = EvoTowerSubscriber()
    
    try:
        rclpy.spin(subscriber)
    except KeyboardInterrupt:
        pass
    
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()