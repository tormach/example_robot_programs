#!/usr/bin/env python3
import rospy
from robot_machine_watchdog_msgs.srv import SetWatchdogMode, SetWatchdogModeRequest
from robot_machine_watchdog_msgs.srv import CheckTorqueSafetyStatus, CheckTorqueSafetyStatusRequest

class RobotMachineWatchdogHandler:
    def __init__(self, node_name='robot_machine_watchdog_handler'):
        if not rospy.core.is_initialized():
            rospy.init_node(node_name, anonymous=True)
        else:
            rospy.loginfo(f"ROS node already initialized. Using existing node for {node_name}")

        self.set_watchdog_service_name = 'set_robot_machine_watchdog_mode'
        self.check_safety_service_name = 'check_torque_safety_status'

        rospy.wait_for_service(self.set_watchdog_service_name)
        rospy.wait_for_service(self.check_safety_service_name)

        self.set_watchdog_mode = rospy.ServiceProxy(self.set_watchdog_service_name, SetWatchdogMode)
        self.check_safety_status_service = rospy.ServiceProxy(self.check_safety_service_name, CheckTorqueSafetyStatus)

    def start_watchdog(self, torque_threshold):
        """
        Start the watchdog with the specified torque threshold.
        :param torque_threshold: The torque threshold to use for collision detection
        :return: True if the watchdog was successfully started, False otherwise
        """
        try:
            request = SetWatchdogModeRequest()
            request.active_status = True
            request.torque_thresh = torque_threshold
            response = self.set_watchdog_mode(request)
            if response.success:
                rospy.loginfo(f"Watchdog started with torque threshold: {torque_threshold}")
                return True
            else:
                rospy.logerr("Failed to start watchdog")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def stop_watchdog(self):
        """
        Stop the watchdog.
        :return: True if the watchdog was successfully stopped, False otherwise
        """
        try:
            request = SetWatchdogModeRequest()
            request.active_status = False
            request.torque_thresh = 0  # This value doesn't matter when stopping
            response = self.set_watchdog_mode(request)
            if response.success:
                rospy.loginfo("Watchdog stopped")
                return True
            else:
                rospy.logerr("Failed to stop watchdog")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_safety_status(self):
        """
        Check the current safety status of the watchdog.
        :return: Boolean indicating if it's safe (True) or not (False).
                 Returns False if the service call fails.
        """
        try:
            request = CheckTorqueSafetyStatusRequest()
            response = self.check_safety_status_service(request)
            if response.success:
                status = "safe" if response.safety_status else "unsafe"
                rospy.loginfo(f"Current safety status: {status}")
                return response.safety_status
            else:
                rospy.logerr("Failed to check safety status")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

def main():
    handler = RobotMachineWatchdogHandler()

    # Example usage
    if handler.start_watchdog(20.0):
        rospy.loginfo("Watchdog started successfully")

        # Check safety status
        safety_status = handler.check_safety_status()
        rospy.loginfo(f"Safety status: {'Safe' if safety_status else 'Unsafe'}")

        rospy.sleep(5)  # Run for 5 seconds

        if handler.stop_watchdog():
            rospy.loginfo("Watchdog stopped successfully")
        else:
            rospy.logerr("Failed to stop watchdog")
    else:
        rospy.logerr("Failed to start watchdog")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
