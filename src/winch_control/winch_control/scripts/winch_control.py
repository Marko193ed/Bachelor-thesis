#!/usr/bin/env python3

import rospy
import math
from time import sleep
from std_srvs.srv import Trigger, TriggerResponse
from dynamixel_workbench_msgs.srv import DynamixelCommand, DynamixelCommandRequest

class WinchControlNode:
    """
    A ROS node to control a Dynamixel servo for a winch-like mechanism.
    It exposes a service that, when called, moves the motor to a specified
    angle and then returns it to the starting position using direct service calls.
    """
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('winch_control_node')
        rospy.loginfo("Starting Simplified Winch Control Node")

        # --- Parameters ---
        self.rotation_degrees = rospy.get_param('~rotation_angle', 660.0)
        self.move_time_sec = rospy.get_param('~move_time', 7.0)
        
        # --- Hardcode the Motor ID ---
        # We know the ID is 2 from our winch.yaml file.
        self.motor_id = 2
        
        # --- Service Client ---
        rospy.loginfo("Waiting for the /dynamixel_workbench/dynamixel_command service...")
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        self.dxl_command_proxy = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command',
            DynamixelCommand
        )

        # --- Service Server ---
        self.trigger_srv = rospy.Service('~trigger_winch', Trigger, self.trigger_winch_cb)

        rospy.loginfo("Winch control node is ready.")
        rospy.loginfo(f"Controlling motor with hardcoded ID: {self.motor_id}")
        rospy.loginfo("Call the '~trigger_winch' service to operate the motor.")

    def set_goal_position(self, position_value):
        """Calls the dynamixel_command service to set the motor's Goal_Position."""
        try:
            req = DynamixelCommandRequest()
            req.id = self.motor_id
            req.addr_name = "Goal_Position"
            req.value = int(position_value)
            
            res = self.dxl_command_proxy(req)
            rospy.loginfo(f"Set Goal_Position to {position_value}, success: {res.comm_result}")
            return res.comm_result
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to set position failed: {e}")
            return False

    def trigger_winch_cb(self, req):
        """This function is called when the 'trigger_winch' service is requested."""
        rospy.loginfo("Winch action triggered!")

        target_position_value = int((self.rotation_degrees / 360.0) * 4096)

        rospy.loginfo(f"Winding to {self.rotation_degrees} degrees (value: {target_position_value})")
        self.set_goal_position(target_position_value)
        
        rospy.loginfo(f"Waiting for {self.move_time_sec} seconds...")
        sleep(self.move_time_sec)
        
        rospy.loginfo("Returning to 0 degrees (value: 0)")
        self.set_goal_position(0)
        
        rospy.loginfo(f"Waiting for {self.move_time_sec} seconds...")
        sleep(self.move_time_sec)
        
        rospy.loginfo("Winch action completed.")
        return TriggerResponse(success=True, message="Winch action completed.")

    def run(self):
        # Keeps the node running to listen for service calls.
        rospy.spin()

if __name__ == '__main__':
    try:
        winch_controller = WinchControlNode()
        winch_controller.run()
    except rospy.ROSInterruptException:
        pass
