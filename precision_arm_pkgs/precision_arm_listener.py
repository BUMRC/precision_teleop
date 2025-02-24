#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DXL_ID_LIST = [1, 2, 3, 4, 5]
STRAIGHT_POSITION = 2047

# TODO: Find min and max for big arm
# MAX_POS = 3350
# MIN_POS = 850

# TODO: Get correct values for larger arm motors
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
BAUDRATE              = 57600
PROTOCOL_VERSION      = 2.0
DEVICENAME            = '/dev/cu.usbserial-FT9HD8LL'
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0

class LargerArmController(Node):
    def __init__(self):
        super().__init__('larger_arm_controller')
        
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',  #Probably change this b/c it might conflict with other names
            self.joint_state_callback,
            10)
        
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the Dynamixel port for larger arm.")
            exit(1)
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate for the larger arm's port.")
            exit(1)

        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        self.move_motors_to_straight_slowly()
        self.get_logger().info("Larger Arm Controller node initialized in straight position.")
    
    def move_motors_to_straight_slowly(self):
        for dxl_id in DXL_ID_LIST:
            current_position, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            target_position = STRAIGHT_POSITION
            step = 10 if current_position < target_position else -10
            
            for pos in range(current_position, target_position, step):
                self.packetHandler.write4ByteTxRx(
                    self.portHandler, dxl_id, ADDR_GOAL_POSITION, pos)
                time.sleep(0.05)
            self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
            time.sleep(0.1)
        self.get_logger().info("Larger arm motors moved to straight positions.")
    
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < len(DXL_ID_LIST):
            self.get_logger().error("Received joint state message with insufficient positions.")
            return
        for i, dxl_id in enumerate(DXL_ID_LIST):
            target_position = int(msg.position[i])
            self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
    
    def shutdown(self):
        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()
        self.get_logger().info("Larger arm port closed and torque disabled.")

def main(args=None):
    rclpy.init(args=args)
    node = LargerArmController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt detected. Shutting down larger arm controller...")
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
