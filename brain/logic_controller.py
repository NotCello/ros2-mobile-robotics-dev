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
        self.ROTATION_SPEED = 0.4
        self.CENTER_THRESHOLD = 20
        
        # State variables
        self.cv_image = None
        self.yaw = None
        self.detected_markers = {}
        
        # OpenCV Setup
        self.bridge = CvBridge()
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
        
        # SUBSCRIBERS
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Image, '/camera/image', self.image_callback, 10)
        
        # PUBLISHERS
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.img_pub = self.create_publisher(Image, '/assignment1/result_image', 10)

        self.get_logger().info("Brain Node Initialized.")

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def detect_markers_in_current_frame(self):
        if self.cv_image is None: return []
        
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=None)
        
        results = []
        if ids is not None:
            ids = ids.flatten()
            for i, marker_id in enumerate(ids):
                c = corners[i][0]
                center_x = int(np.mean(c[:, 0]))
                results.append((marker_id, center_x, corners[i]))
        return results

    def perform_task(self):
        # 1. WAIT FOR ODOM
        while self.yaw is None and rclpy.ok():
            self.get_logger().info("Waiting for sensors...", throttle_duration_sec=2)
            rclpy.spin_once(self, timeout_sec=1)

        if self.yaw is None: return

        # 2. ROTATION PHASE (SCANNING)
        self.get_logger().info("Starting 360 scan...")
        cmd = Twist()
        cmd.angular.z = self.ROTATION_SPEED
        
        last_yaw = self.yaw
        start_time = time.time()
        
        while len(self.detected_markers) < self.EXPECTED_MARKERS and (time.time() - start_time) < 60.0 and rclpy.ok():
            self.vel_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.01)
            
            if self.yaw is not None:
                current_yaw = self.yaw
                last_yaw = current_yaw

            detections = self.detect_markers_in_current_frame()
            for mid, cx, _ in detections:
                if mid not in self.detected_markers:
                    self.get_logger().info(f"Found new marker: {mid}")
                    self.detected_markers[mid] = self.yaw 

        self.vel_pub.publish(Twist()) # Stop scanning
        self.get_logger().info(f"Scan Complete. Found: {sorted(self.detected_markers.keys())}")
        time.sleep(1.0) # Let physics settle
        
        # 3. ALIGNMENT PHASE
        sorted_ids = sorted(self.detected_markers.keys())
        
        for target_id in sorted_ids:
            self.get_logger().info(f"Approaching Marker {target_id}...")
            
            # --- PHASE 3A: COARSE ALIGN ---
            target_yaw = self.detected_markers[target_id]
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                if self.yaw is None: continue

                error_yaw = self.normalize_angle(target_yaw - self.yaw)
                
                # IMPROVEMENT: Tighter threshold (0.2 -> 0.1)
                if abs(error_yaw) < 0.1: 
                    self.vel_pub.publish(Twist()) # Stop before fine align
                    time.sleep(0.5) 
                    break
                
                # Proportional control for coarse align
                cmd.angular.z = 0.5 * error_yaw
                self.vel_pub.publish(cmd)

            # --- PHASE 3B: FINE ALIGN ---
            self.get_logger().info(f"Fine aligning marker {target_id}...")
            
            # Safety counter: if we oscillate too long, just stop
            alignment_start_time = time.time()
            
            while rclpy.ok():
                # Timeout safety: If we can't align in 10 seconds, give up and shoot
                if (time.time() - alignment_start_time) > 10.0:
                     self.get_logger().warn(f"Alignment timed out for {target_id}")
                     break

                rclpy.spin_once(self, timeout_sec=0.01)
                detections = self.detect_markers_in_current_frame()
                target_data = next((d for d in detections if d[0] == target_id), None)
                
                if target_data:
                    mid, cx, corners = target_data
                    h, w, _ = self.cv_image.shape
                    error_x = (w / 2) - cx
                    
                    # 1. CHECK THRESHOLD FIRST (Exit condition)
                    # We use 15 pixels as a balanced threshold
                    if abs(error_x) < 15:
                        self.vel_pub.publish(Twist()) # Hard Stop
                        self.get_logger().info(f"Marker {target_id} Centered!")
                        
                        # DRAW and PUBLISH
                        import cv2 
                        c = corners[0]
                        center = (int(np.mean(c[:, 0])), int(np.mean(c[:, 1])))
                        cv2.circle(self.cv_image, center, 50, (0, 255, 0), 3)
                        out_msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
                        self.img_pub.publish(out_msg)
                        time.sleep(2)
                        break

                    # 2. CALCULATE SPEED (Robust P-Controller)
                    Kp = 0.002  # Reverted to stronger gain
                    angular_z = Kp * error_x
                    
                    # 3. APPLY LIMITS (Min/Max Speed)
                    # Max speed: 0.15 (prevent shooting past)
                    # Min speed: 0.05 (prevent stalling/stiction)
                    if abs(angular_z) > 0.15:
                        angular_z = math.copysign(0.15, angular_z)
                    elif abs(angular_z) < 0.05:
                        angular_z = math.copysign(0.05, angular_z)
                    
                    cmd.angular.z = angular_z
                    self.vel_pub.publish(cmd)
                    
                else:
                    # Smart Search Fallback
                    # Reduced speed to 0.05 to match the fine-align minimum speed
                    if self.yaw is not None:
                        error_yaw = self.normalize_angle(target_yaw - self.yaw)
                        cmd.angular.z = math.copysign(0.05, error_yaw)
                    else:
                        cmd.angular.z = 0.05
                    
                    self.vel_pub.publish(cmd)

        self.get_logger().info("Task Finished.")
        self.vel_pub.publish(Twist())
        while rclpy.ok():
            time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        # EXECUTE THE TASK HERE, AFTER INIT IS DONE
        node.perform_task()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()