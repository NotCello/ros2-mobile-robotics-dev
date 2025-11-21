#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data

class AssignmentNode(Node):
    def __init__(self):
        super().__init__('assignment_logic_node')
        
        # --- 1. SETUP ---
        self.br = CvBridge()

        # CRITICAL FIX: Use 'qos_profile_sensor_data' (Best Effort). 
        # If you use '10', the node ignores Gazebo images!
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            qos_profile_sensor_data
        )
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.evidence_pub = self.create_publisher(Image, '/assignment/marker_evidence', 10)

        # ArUco Setup
        # Ensure this matches your generator script!
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # --- 2. VARIABLES ---
        self.state = "SEARCHING"
        self.seen_ids = []       
        self.target_list = []    
        self.current_target = None 
        self.k_p = 0.002         

        self.get_logger().info("Node started! Waiting for camera data...")

    def image_callback(self, data):
        # DEBUG: Confirm we are actually receiving data
        # self.get_logger().info("Image received!", throttle_duration_sec=5.0)

        # A. Convert ROS Image to OpenCV
        try:
            current_frame = self.br.imgmsg_to_cv2(data, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # [cite_start]B. Detect Markers [cite: 1165-1168]
        corners, ids, _ = cv2.aruco.detectMarkers(
            current_frame, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        # DEBUG: If we see ANY ID, print it immediately
        if ids is not None:
            self.get_logger().info(f"Saw Marker IDs: {ids.flatten()}", throttle_duration_sec=1.0)

        height, width, _ = current_frame.shape
        image_center_x = width / 2
        cmd = Twist()

        # --- C. STATE MACHINE ---
        
        if self.state == "SEARCHING":
            # [cite_start]Rotate to find all 5 markers [cite: 3226-3227]
            cmd.angular.z = 0.3

            if ids is not None:
                for i in ids.flatten():
                    if i not in self.seen_ids:
                        self.seen_ids.append(int(i))
                        self.get_logger().info(f"New unique marker found: {i}")

            # Transition condition
            if len(self.seen_ids) >= 5: 
                cmd.angular.z = 0.0 
                self.state = "PLANNING"
                self.get_logger().info("All 5 markers found! Switching to PLANNING.")

        elif self.state == "PLANNING":
            # [cite_start]Sort ascending [cite: 3229]
            self.seen_ids.sort()
            self.target_list = self.seen_ids.copy() 
            
            self.current_target = self.target_list.pop(0)
            self.state = "CENTERING"
            self.get_logger().info(f"Targeting Marker ID: {self.current_target}")

        elif self.state == "CENTERING":
            marker_found_in_frame = False
            
            if ids is not None:
                if self.current_target in ids.flatten():
                    marker_found_in_frame = True
                    index = list(ids.flatten()).index(self.current_target)
                    
                    # Calculate center
                    c = corners[index][0] 
                    marker_center_x = np.mean(c[:, 0])
                    marker_center_y = np.mean(c[:, 1])

                    # P-Controller
                    error = image_center_x - marker_center_x
                    cmd.angular.z = self.k_p * error
                    
                    # Centered Threshold
                    if abs(error) < 10:
                        cmd.angular.z = 0.0
                        self.get_logger().info(f"ID {self.current_target} Centered!")
                        
                        # --- PUBLISH EVIDENCE ---
                        cv2.circle(current_frame, (int(marker_center_x), int(marker_center_y)), 
                                   50, (0, 255, 0), 4) 
                        
                        evidence_msg = self.br.cv2_to_imgmsg(current_frame, "bgr8")
                        self.evidence_pub.publish(evidence_msg)
                        self.get_logger().info(f"Published evidence for ID {self.current_target}")
                        
                        if len(self.target_list) > 0:
                            self.current_target = self.target_list.pop(0)
                            self.get_logger().info(f"Next target: {self.current_target}")
                            cmd.angular.z = 0.5 # Search kick
                        else:
                            self.state = "DONE"
                            self.get_logger().info("Assignment Completed!")

            # Lost target? Rotate to find it
            if not marker_found_in_frame:
                cmd.angular.z = 0.3

        elif self.state == "DONE":
            cmd.angular.z = 0.0

        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AssignmentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()