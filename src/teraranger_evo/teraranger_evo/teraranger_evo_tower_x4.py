#!/usr/bin/env python3
###### TeraRanger Evo Tower X4 Data Processor #######
# #
# Based on TeraRanger Example Code STD           #
# #
############ www.terabee.com #################
import serial
import serial.tools.list_ports
import sys
import crcmod.predefined  # To install: pip install crcmod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class EvoTowerX4ROS(Node):
    def __init__(self):
        super().__init__('teraranger_evo_tower_x4')
        
        # Create publisher for distance data
        self.distance_pub = self.create_publisher(
            Float32MultiArray,
            '/teraranger_evo/distances',
            10
        )
        
        # Initialize distance array [front, right, back, left]
        self.distances = [0.0, 0.0, 0.0, 0.0]
        
        # Find and setup the Evo device
        self.setup_evo()
        
        # Create a timer for the main processing loop
        self.timer = self.create_timer(0.01, self.run)  # 100Hz
        
    def setup_evo(self):
        """Find and configure the Evo device"""
        self.get_logger().info('Starting Evo Tower X4 data streaming - Alternating Sensors Mode')
        
        # Get the port the evo has been connected to
        port = self.findEvo()
        if port == 'NULL':
            self.get_logger().error("Sorry couldn't find the Evo. Exiting.")
            sys.exit()
        else:
            self.evo = self.openEvo(port)
            
            # First set Tower mode
            self.setTowerMode(self.evo)
            
            # Enable data streaming
            self.enableDataStreaming(self.evo)
            
            # Then set Binary mode for reading data
            self.setBinaryMode(self.evo)

    def findEvo(self):
        # Find Live Ports, return port name if found, NULL if not
        self.get_logger().info('Scanning all live ports on this PC')
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # Check if the TeraRanger device is connected (USB VID:PID identifier)
            if "5740" in p[2]:
                self.get_logger().info(f'Evo found on port {p[0]}')
                return p[0]
        return 'NULL'

    def openEvo(self, portname):
        self.get_logger().info('Attempting to open port...')
        # Open the Evo and catch any exceptions thrown by the OS
        self.get_logger().info(portname)
        evo = serial.Serial(portname, baudrate=115200, timeout=2)
        self.get_logger().info('Serial port opened')
        return evo

    def setTowerMode(self, evo):
        # Send the command for Tower mode: 00 31 03 E5
        tower_mode = bytes([0x00, 0x31, 0x03, 0xE5])
        self.get_logger().info('Setting sensor to Tower mode...')
        # Flush input buffer
        evo.flushInput()
        # Write the tower mode command to the Evo
        evo.write(tower_mode)
        # Flush output buffer
        evo.flushOutput()
        self.get_logger().info('Tower mode command sent')

    def enableDataStreaming(self, evo):
        # Send the command to enable data streaming: 00 52 02 01 DF
        stream_data = bytes([0x00, 0x52, 0x02, 0x01, 0xDF])
        self.get_logger().info('Enabling data streaming...')
        # Flush input buffer
        evo.flushInput()
        # Write the data streaming command to the Evo
        evo.write(stream_data)
        # Flush output buffer
        evo.flushOutput()
        self.get_logger().info('Data streaming enabled')

    def setBinaryMode(self, evo):
        # Send the command for Binary mode
        set_bin = bytes([0x00, 0x11, 0x02, 0x4C])
        self.get_logger().info('Setting sensor to Binary mode...')
        # Flush input buffer
        evo.flushInput()
        # Write the binary command to the Evo
        evo.write(set_bin)
        # Flush output buffer
        evo.flushOutput()
        self.get_logger().info('Binary mode command sent')

    def process_x4_data(self, evo_serial):
        """
        Process data from the TeraRanger Evo Tower X4 LIDAR
        Reading only alternating sensors (1, 3, 5, 7) as per actual connection pattern
        Data format (20 bytes):
        - TH (2 bytes header)
        - Distance readings (2 bytes per sensor x 8 sensors = 16 bytes)
        - Mask (1 byte)
        - CRC8 (1 byte)
        """
        crc8_fn = crcmod.predefined.mkPredefinedCrcFun('crc-8')
        
        # Read first byte to find the header 'T'
        first_byte = evo_serial.read(1)
        if not first_byte or first_byte != b'T':
            return "Waiting for frame header 'T'", None
        
        # Read the second byte, should be 'H'
        second_byte = evo_serial.read(1)
        if not second_byte or second_byte != b'H':
            return "Invalid header: missing 'H'", None
        
        # Read the rest of the frame (18 bytes: 16 for distances, 1 for mask, 1 for CRC)
        frame_data = evo_serial.read(18)
        if len(frame_data) < 18:
            return "Incomplete frame data", None
        
        # Complete frame
        frame = first_byte + second_byte + frame_data
        
        # Verify CRC
        if frame[-1] != crc8_fn(frame[:-1]):
            return "CRC mismatch. Check connection or make sure only one program accesses the sensor port.", None
        
        # Extract the mask byte
        mask = frame[-2]
        
        # Process the distance readings for alternating sensors (positions 1, 3, 5, 7)
        distances = []
        sensor_status = []
        active_sensors_indices = [0, 2, 4, 6]  # Indices for sensors in positions 1, 3, 5, 7
        
        for i, sensor_idx in enumerate(active_sensors_indices):
            # Get 2 bytes for each sensor
            msb = frame[2 + (sensor_idx * 2)]
            lsb = frame[3 + (sensor_idx * 2)]
            
            # Combine the bytes to get the distance in mm
            distance_mm = (msb << 8) | lsb
            
            # Check special cases
            if distance_mm == 65535:  # Sensor measuring above maximum limit
                distances.append(float('inf'))
            elif distance_mm == 1:  # Sensor not able to measure
                distances.append(float('nan'))
            elif distance_mm == 0:  # Sensor detecting object below minimum range
                distances.append(float('-inf'))
            else:
                # Convert to meters
                distances.append(distance_mm / 1000.0)
            
            # Check mask to see if readings are new
            if mask & (1 << sensor_idx):
                sensor_status.append("new")
            else:
                sensor_status.append("old")
        
        # Update the distances array [front, right, back, left]
        self.distances = distances
        
        # Format the output
        result = "TeraRanger Evo Tower X4 Readings (Alternating Sensors):\n"
        for i in range(4):
            result += f"Sensor {i+1} (Position {active_sensors_indices[i]+1}): {distances[i]:.3f}m ({sensor_status[i]})\n"
        
        return result, distances

    def publish_distances(self, distances):
        """Publish the distances as a Float32MultiArray message"""
        if distances is not None:
            msg = Float32MultiArray()
            msg.data = distances
            self.distance_pub.publish(msg)

    def run(self):
        try:
            result, distances = self.process_x4_data(self.evo)
            if result:
                self.get_logger().info(result)
            
            if distances is not None:
                self.publish_distances(distances)
                
        except Exception as e:
            self.get_logger().error(f"Error in processing data: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        evo_ros = EvoTowerX4ROS()
        rclpy.spin(evo_ros)
    except KeyboardInterrupt:
        pass
    except serial.serialutil.SerialException:
        evo_ros.get_logger().error("Device disconnected (or multiple access on port). Exiting...")
    finally:
        evo_ros.get_logger().info("Closing connection...")
        if 'evo' in evo_ros.__dict__:
            evo_ros.evo.close()
        evo_ros.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == "__main__":
    main()
