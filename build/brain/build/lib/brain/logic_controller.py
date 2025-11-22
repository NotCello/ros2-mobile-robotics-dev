import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import time
import numpy as np

class BrainNode(Node):
    def __init__(self):
        super().__init__('brain_node')
        
        # Configuration
        self.EXPECTED_MARKERS = 5
        self.ROTATION_SPEED = 0.3
        self.CENTER_THRESHOLD = 10  # Pixels
        
        # State variables
        self.cv_image = None
        self.yaw = None
        self.detected_markers = {}  # {id: (yaw, score)}
        
        # OpenCV Setup
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        # Fix for Ubuntu 24.04 / OpenCV 4.x (Friend uses _create(), we use constructor or None)
        self.aruco_params = aruco.DetectorParameters() 

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        
        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/assignment1/result_image', 10)

        self.get_logger().info("Brain Node Initialized. Waiting for sensors...")

    def odom_callback(self, msg):
        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        try:
            # Just save the image, don't process logic here!
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def detect_markers_in_current_frame(self):
        """Helper to find markers in the currently saved image"""
        if self.cv_image is None: return []
        
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        # Using None for parameters is the safest way to avoid SegFaults on Jazzy
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=None)
        
        results = []
        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                # Calculate center x
                c = corners[i][0]
                center_x = int(np.mean(c[:, 0]))
                results.append((marker_id, center_x, corners[i]))
        return results

    def perform_task(self):
        """Main Sequential Logic (Like start_scanning.py)"""
        
        # 1. WAIT FOR ODOM
        while self.yaw is None and rclpy.ok():
            self.get_logger().info("Waiting for Odom...", throttle_duration_sec=2)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 2. ROTATION PHASE (Scan 360)
        self.get_logger().info("Starting 360 scan...")
        cmd = Twist()
        cmd.angular.z = self.ROTATION_SPEED
        
        start_yaw = self.yaw
        turned_angle = 0.0
        last_yaw = self.yaw

        # Loop until we turn 360 OR find enough markers
        while turned_angle < (2 * math.pi + 0.2) and len(self.detected_markers) < self.EXPECTED_MARKERS:
            self.vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # Update rotation tracking
            delta = self.normalize_angle(self.yaw - last_yaw)
            turned_angle += abs(delta)
            last_yaw = self.yaw

            # Scan for markers
            detections = self.detect_markers_in_current_frame()
            for mid, cx, _ in detections:
                if mid not in self.detected_markers:
                    self.get_logger().info(f"Found new marker: {mid}")
                    # Store marker ID and its approximate yaw relative to robot
                    self.detected_markers[mid] = self.yaw 

        # Stop
        self.vel_pub.publish(Twist())
        self.get_logger().info(f"Scan Complete. Found: {sorted(self.detected_markers.keys())}")
        
        # 3. ALIGNMENT PHASE
        sorted_ids = sorted(self.detected_markers.keys())
        
        for target_id in sorted_ids:
            self.get_logger().info(f"Approaching Marker {target_id}...")
            
            # A. Turn towards approximate location first (Coarse Align)
            # This prevents the robot from getting lost if the marker isn't immediately visible
            target_yaw = self.detected_markers[target_id]
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                error_yaw = self.normalize_angle(target_yaw - self.yaw)
                
                if abs(error_yaw) < 0.1: # Roughly aligned
                    break
                
                cmd.angular.z = 0.5 * error_yaw
                self.vel_pub.publish(cmd)

            # B. Visual Servoing (Fine Align & Center)
            self.get_logger().info(f"Fine aligning marker {target_id}...")
            marker_visible_count = 0
            
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.05)
                detections = self.detect_markers_in_current_frame()
                
                # Find our target in current frame
                target_data = next((d for d in detections if d[0] == target_id), None)
                
                if target_data:
                    mid, cx, corners = target_data
                    h, w, _ = self.cv_image.shape
                    error_x = (w / 2) - cx
                    
                    # P-Controller
                    cmd.angular.z = 0.03 * error_x
                    self.vel_pub.publish(cmd)
                    
                    if abs(error_x) < self.CENTER_THRESHOLD:
                        self.vel_pub.publish(Twist()) # Stop
                        self.get_logger().info(f"Marker {target_id} Centered!")
                        
                        # C. DRAW AND PUBLISH (The Assignment Goal)
                        # Draw Green Circle
                        c = corners[0]
                        center = (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))
                        cv2.circle(self.cv_image, center, 1050, (0, 255, 0), 3)
                        cv2.putText(self.cv_image, f"ID {target_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        self.get_logger().info(f"Drawed {target_id}Circle!")
                        
                        # Publish result
                        out_msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
                        self.img_pub.publish(out_msg)
                        time.sleep(2) # Show it for 2 seconds
                        break # Move to next marker
                else:
                    # If lost, rotate slowly to find it
                    cmd.angular.z = 0.2
                    self.vel_pub.publish(cmd)

        self.get_logger().info("All markers processed. Task Finished.")

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        node.perform_task()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()