#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState
import time
import sys, select, tty, termios

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DXL_ID_LIST = [1, 2, 3, 4, 5]
STRAIGHT_POSITION = 2047

MAX_POS = 3350
MIN_POS = 850

ADDR_HOMING_OFFSET    = 20
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
ADDR_PRESENT_POSITION = 132
ADDR_OPERATING_MODE   = 11  
BAUDRATE              = 57600
PROTOCOL_VERSION      = 2.0
DEVICENAME            = '/dev/tty0'  # Change this!
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0
OPERATING_MODE        = 4
WASD_GRANULARITY = 10

class PrecisionArmBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamixel_arm_controller')
        
        self.publisher = self.create_publisher(Int32MultiArray, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        
        # Set terminal to cbreak (non-blocking) mode for keyboard input
        self.stdin_fd = sys.stdin.fileno()
        self.old_term_settings = termios.tcgetattr(self.stdin_fd)
        tty.setcbreak(self.stdin_fd)
        
        if not self.portHandler.openPort():
            self.get_logger().error("Failed to open the Dynamixel port.")
            exit(1)
        if not self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().error("Failed to set baudrate for the Dynamixel port.")
            exit(1)
            
        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        
        self.move_motors_to_straight()
        self.zero_motors()
        
        self.joint_names = [f'joint{i}' for i in DXL_ID_LIST] + ['fake_joint1', 'fake_joint2']
        self.fake_motor_positions = [0, 0]
        
        self.get_logger().info("Dynamixel Arm Controller node initialized.")
    
    # THESE STRAIGHT POSITIONS AREN'T ACTUALLY CORRECT, DO IT MANUALLY FOR NOW
    def move_motors_to_straight(self):
        for dxl_id in DXL_ID_LIST:
            current_position, _, _ = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            target_position = STRAIGHT_POSITION
            step = 10 if current_position < target_position else -10
            
            for pos in range(current_position, target_position, step):
                self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, pos)
                time.sleep(0.05)
            self.packetHandler.write4ByteTxRx(self.portHandler, dxl_id, ADDR_GOAL_POSITION, target_position)
            time.sleep(0.1)
        self.get_logger().info("Motors moved to straight positions.")
    
    def zero_motors(self):
        for dxl_id in DXL_ID_LIST:
            # STEP 1: Disable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"Torque disabled for ID {dxl_id} for Operating Mode adjustment.")
            time.sleep(0.1)
            
            # STEP 2: Set Operating Mode to Extended Position Control (multi-turn)
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_OPERATING_MODE, OPERATING_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"Operating mode set to Extended Position Control for ID {dxl_id}.")
            time.sleep(0.1)
            
            # STEP 3: Read the current position
            present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"Current Present Position for ID {dxl_id}: {present_position}")
            
            # STEP 4: Set the new Homing Offset (using Extended Position Control, larger values are allowed)
            new_homing_offset = -present_position
            self.get_logger().info(f"Setting Homing Offset for ID {dxl_id} to: {new_homing_offset}")
            
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, dxl_id, ADDR_HOMING_OFFSET, new_homing_offset)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"Homing Offset successfully set for ID {dxl_id}.")
            
            # STEP 5: Re-enable Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                self.get_logger().error(self.packetHandler.getRxPacketError(dxl_error))
            else:
                self.get_logger().info(f"Torque re-enabled for ID {dxl_id}.")
    
    def get_key_nonblocking(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None
    

    def timer_callback(self):
        msg = Int32MultiArray()
        msg.data = []

        for dxl_id in DXL_ID_LIST:
            pos, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, dxl_id, ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(
                    f"Error reading position for ID {dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            msg.data.append(pos)

        key = self.get_key_nonblocking()
        if key:
            if key.lower() == 'w':
                self.fake_motor_positions[0] += WASD_GRANULARITY
            elif key.lower() == 's':
                self.fake_motor_positions[0] -= WASD_GRANULARITY
            elif key.lower() == 'a':
                self.fake_motor_positions[1] += WASD_GRANULARITY
            elif key.lower() == 'd':
                self.fake_motor_positions[1] -= WASD_GRANULARITY

        msg.data.extend([self.fake_motor_positions[0], self.fake_motor_positions[1]])
        self.publisher.publish(msg)

    
    def shutdown(self):
        for dxl_id in DXL_ID_LIST:
            self.packetHandler.write1ByteTxRx(self.portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.portHandler.closePort()
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_term_settings)
        self.get_logger().info("Dynamixel port closed and torque disabled.")

def main(args=None):
    rclpy.init(args=args)
    node = PrecisionArmBroadcaster()
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
