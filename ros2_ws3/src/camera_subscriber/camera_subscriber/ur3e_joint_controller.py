import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import threading
import math
from rclpy.time import Time

class UR3eJointController(Node):
	def __init__(self):
		super().__init__('ur3e_joint_controller')

		self.publisher_ = self.create_publisher(JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10)
		self.target_position = 0.0
		self.current_position = None 

		# Timer wywołujący co 1 sekundę
		self.timer = self.create_timer(1.0, self.timer_callback)
		self.get_logger().info("UR3e Joint Controller initialized. Timer active.")

	def timer_callback(self):
		
		if self.target_position != self.current_position:
		    self.get_logger().info(f"Timer callback triggered. Current position: {self.target_position:.2f} rad")
		    self.publish_joint_position(self.target_position)
		    self.current_position = self.target_position 
		else:
		    self.get_logger().info("No change in position. Skipping publication.")


	def publish_joint_position(self, position):
		
		trajectory_msg = JointTrajectory()
		trajectory_msg.joint_names = [
		'shoulder_pan_joint',      
		'shoulder_lift_joint',      
		'elbow_joint',
		'wrist_1_joint',
		'wrist_2_joint',
		'wrist_3_joint'
		]

		
		point = JointTrajectoryPoint()
		point.positions = [
		position,                   
		0.0,                        
		0.0,
		0.0,
		0.0,
		0.0
		]
		point.time_from_start.sec = 4  

		current_time = self.get_clock().now()
		trajectory_msg.header.stamp = current_time.to_msg()

		trajectory_msg.points.append(point)

		self.publisher_.publish(trajectory_msg)
		self.get_logger().info(f"Published joint trajectory with position: {position:.2f} rad")



def run_ui(node):
    width, height = 800, 600
    center_y = height // 2

    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    canvas[:] = (0, 0, 0)  # Czarny kolor
    cv2.line(canvas, (0, center_y), (width, center_y), (255, 255, 255), 1)  # Środkowa pozioma linia

    def on_mouse_event(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
 
            max_angle_deg = 90.0  # Zakres w stopniach
            max_angle_rad = math.radians(max_angle_deg)  # Zakres w radianach
            
            position = ((center_y - y) / (height // 2)) * max_angle_rad
            
            print(f'Kliknięcie w pozycji: y={y}, obrót={position:.2f} rad')
            
            temp_canvas = canvas.copy()
            cross_size = 10
            cv2.line(temp_canvas, (x - cross_size, y - cross_size), (x + cross_size, y + cross_size), (0, 0, 255), 2)
            cv2.line(temp_canvas, (x - cross_size, y + cross_size), (x + cross_size, y - cross_size), (0, 0, 255), 2)
            
            cv2.imshow('OKNO STEROWANIA', temp_canvas)
            
            node.publish_joint_position(position)

    cv2.namedWindow('OKNO STEROWANIA')
    cv2.setMouseCallback('OKNO STEROWANIA', on_mouse_event)

    while True:
        cv2.imshow('OKNO STEROWANIA', canvas)
        key = cv2.waitKey(10)
        if key == 27:  # ESC - wyjście
            break

    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = UR3eJointController()
    
    run_ui(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down UR3e Joint Controller...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

