#!/usr/bin/env python3
"""
A ROS2 Humble node that integrates Dynamixel motor control with ROS2.
This node:
  • Initializes the Dynamixel motors using the Dynamixel SDK.
  • Moves the motors to predefined straight positions (2047) slowly.
  • Keeps the motors enabled and continuously reads their current positions.
  • Publishes the joint positions as sensor_msgs/JointState for use
    (e.g., as a controller for a larger-scale robotic arm).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DXL_ID_LIST = [1, 2, 3, 4, 5]
STRAIGHT_POSITION = 2047

MAX_POS = 3350
MIN_POS = 850

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
BAUDRATE              = 57600
PROTOCOL_VERSION      = 2.0
DEVICENAME            = '/dev/cu.usbserial-FT9HD8LL'
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0

class DynamixelArmController(Node):
    def __init__(self):
        super().__init__('dynamixel_arm_controller')
        
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  #seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the Dynamixel port.")
            exit(1)
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate for the Dynamixel port.")
            exit(1)
            
        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        self.move_motors_to_straight_slowly()
        self.joint_names = [f'joint{i}' for i in DXL_ID_LIST]
        self.get_logger().info("Dynamixel Arm Controller node initialized.")
    
    def move_motors_to_straight_slowly(self):
        for dxl_id in DXL_ID_LIST:
            current_position, _, _ = self.packetHandler.read4ByteTxRx(self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            target_position = STRAIGHT_POSITION
            step = 10 if current_position < target_position else -10
            
            for pos in range(current_position, target_position, step):
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pos)
                time.sleep(0.05)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
            time.sleep(0.1)
        self.get_logger().info("Motors moved to straight positions slowly.")
    
    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        positions = []
        
        for dxl_id in DXL_ID_LIST:
            pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(
                    f"Error reading position for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            positions.append(float(pos))
        
        msg.position = positions
        self.publisher_.publish(msg)
    
    def shutdown(self):
        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()
        self.get_logger().info("Dynamixel port closed and torque disabled.")

def main(args=None):
    rclpy.init(args=args)
    node = DynamixelArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down node...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
