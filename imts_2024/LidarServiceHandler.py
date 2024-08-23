#!/usr/bin/env python3
# FILE: LidarServiceHandler.py
import rospy
import numpy as np
from lidar_pointcloud_processor_msgs.srv import ProcessLidarData, EstimateLidarLine, RefineLidarLengthDisplacement, PurgeLidarData, FindHighestLidarZ, FindClosestLidarRectangle
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

class LidarServiceHandler:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(LidarServiceHandler, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._initialized = True

        self._init_ros_node()

        # Service clients
        self.process_pointclouds_service = '/process_pointclouds'
        self.find_line_service = '/find_line'
        self.refine_length_displacement_service = '/refine_length_displacement'
        self.purge_lidar_data_service = '/purge_lidar_data'
        self.find_highest_cloud_point_service = '/find_highest_cloud_point'
        self.find_closest_rectangle_service = '/find_closest_rectangle'  # New service

        rospy.wait_for_service(self.process_pointclouds_service)
        rospy.wait_for_service(self.find_line_service)
        rospy.wait_for_service(self.refine_length_displacement_service)
        rospy.wait_for_service(self.purge_lidar_data_service)
        rospy.wait_for_service(self.find_highest_cloud_point_service)
        rospy.wait_for_service(self.find_closest_rectangle_service)  # Wait for new service

        self.process_pointclouds = rospy.ServiceProxy(self.process_pointclouds_service, ProcessLidarData)
        self.find_line = rospy.ServiceProxy(self.find_line_service, EstimateLidarLine)
        self.refine_length_displacement = rospy.ServiceProxy(self.refine_length_displacement_service, RefineLidarLengthDisplacement)
        self.purge_lidar_data = rospy.ServiceProxy(self.purge_lidar_data_service, PurgeLidarData)
        self.find_highest_cloud_point = rospy.ServiceProxy(self.find_highest_cloud_point_service, FindHighestLidarZ)
        self.find_closest_rectangle = rospy.ServiceProxy(self.find_closest_rectangle_service, FindClosestLidarRectangle)  # New service


        # Publisher for lidar state
        self.state_publisher = rospy.Publisher('lidar_status', Bool, queue_size=10)

    def _init_ros_node(self):
        """Initialize ROS node if it hasn't been initialized yet."""
        try:
            # Check if the node has already been initialized
            rospy.get_name()
        except rospy.exceptions.ROSException:
            # If not, initialize the node
            rospy.init_node('lidar_service_handler', anonymous=True)
            rospy.loginfo("Initialized ROS node: lidar_service_handler")
        else:
            rospy.loginfo("ROS node already initialized. Using existing node.")

    def call_process_pointclouds(self, min_z, max_z, cluster_eps=0.02,
                                 enable_plotting_data=False,
                                 save_figure_to_bitmap_headless=True,
                                 purge_after_processing=True):
        try:
            response = self.process_pointclouds(
                min_z=min_z,
                max_z=max_z,
                cluster_eps=cluster_eps,
                enable_plotting_data=enable_plotting_data,
                save_figure_to_bitmap_headless=save_figure_to_bitmap_headless,
                purge_after_processing=purge_after_processing
            )
            if response.success:
                rospy.loginfo("Service call successful")
                for i, pose in enumerate(response.object_poses):
                    rospy.loginfo(f"Object {i + 1} pose:")
                    rospy.loginfo(f"  Position: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}")
                    rospy.loginfo(f"  Orientation: x={pose.orientation.x:.4f}, y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}")
                return response.object_poses
            else:
                rospy.logwarn("Service call failed")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def call_find_line_service(self, line_id, min_z, max_z, cluster_eps=0.02, enable_plotting_data=False, save_figure_to_bitmap_headless=True, purge_after_processing=True):
        try:
            response = self.find_line(
                line_id=line_id,
                min_z=min_z,
                max_z=max_z,
                cluster_eps=cluster_eps,
                enable_plotting_data=enable_plotting_data,
                save_figure_to_bitmap_headless=save_figure_to_bitmap_headless,
                purge_after_processing=purge_after_processing
            )
            if response.success:
                rospy.loginfo("Find line service call successful")
                matrix = self.pose_to_matrix(response.object_pose)
                rospy.loginfo("4x4 Homogeneous Transformation Matrix:")
                rospy.loginfo(f"\n{matrix}")
                return response.object_pose
            else:
                rospy.logwarn("Find line service call was not successful")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Find line service call failed: {e}")
            return None

    def call_refine_length_displacement_service(self, min_z, max_z, object_length, search_direction, initial_object_pose, cluster_eps=0.02, enable_plotting_data=False, save_figure_to_bitmap_headless=True, purge_after_processing=True):
        try:
            response = self.refine_length_displacement(
                min_z=min_z,
                max_z=max_z,
                object_length=object_length,
                search_direction=search_direction,
                initial_object_pose=initial_object_pose,
                cluster_eps=cluster_eps,
                enable_plotting_data=enable_plotting_data,
                save_figure_to_bitmap_headless=save_figure_to_bitmap_headless,
                purge_after_processing=purge_after_processing
            )
            if response.success:
                rospy.loginfo("Refine length displacement service call successful")
                matrix = self.pose_to_matrix(response.refined_object_pose)
                rospy.loginfo("Refined 4x4 Homogeneous Transformation Matrix:")
                rospy.loginfo(f"\n{matrix}")
                return response.refined_object_pose
            else:
                rospy.logwarn("Refine length displacement service call was not successful")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Refine length displacement service call failed: {e}")
            return None


    def call_find_highest_cloud_point(self):
        try:
            self.lidar_on()  # Turn on the lidar
            rospy.sleep(0.1)  # Short delay to ensure lidar is on

            response = self.find_highest_cloud_point()

            self.lidar_off()  # Turn off the lidar after getting the data

            if response.success:
                rospy.loginfo(f"Highest cloud point found at Z = {response.highest_z:.4f}")
                return response.highest_z
            else:
                rospy.logwarn("Failed to find highest cloud point")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Find highest cloud point service call failed: {e}")
            self.lidar_off()  # Ensure lidar is turned off even if service call fails
            return None


    def call_purge_lidar_data(self):
        try:
            response = self.purge_lidar_data()
            if response.success:
                rospy.loginfo("Lidar data purged successfully")
            else:
                rospy.logwarn("Failed to purge lidar data")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Purge lidar data service call failed: {e}")
            return False

    def call_find_closest_rectangle_service(self, min_z, max_z, cluster_eps, rectangle_width, rectangle_height, initial_pose, enable_plotting_data=False, save_figure_to_bitmap_headless=True, purge_after_processing=True):
        try:
            response = self.find_closest_rectangle(
                min_z=min_z,
                max_z=max_z,
                cluster_eps=cluster_eps,
                rectangle_width=rectangle_width,
                rectangle_height=rectangle_height,
                initial_pose=initial_pose,
                enable_plotting_data=enable_plotting_data,
                save_figure_to_bitmap_headless=save_figure_to_bitmap_headless,
                purge_after_processing=purge_after_processing
            )
            if response.success:
                rospy.loginfo("Find closest rectangle service call successful")
                matrix = self.pose_to_matrix(response.refined_pose)
                rospy.loginfo("Refined 4x4 Homogeneous Transformation Matrix:")
                rospy.loginfo(f"\n{matrix}")
                return response.refined_pose
            else:
                rospy.logwarn("Find closest rectangle service call was not successful")
                return None
        except rospy.ServiceException as e:
            rospy.logerr(f"Find closest rectangle service call failed: {e}")
            return None

    @staticmethod
    def quaternion_to_rotation_matrix(q):
        """Convert a quaternion to a rotation matrix."""
        x, y, z, w = q
        return np.array([
            [1 - 2*y*y - 2*z*z,     2*x*y - 2*z*w,     2*x*z + 2*y*w],
            [    2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z,     2*y*z - 2*x*w],
            [    2*x*z - 2*y*w,     2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]
        ])

    @staticmethod
    def pose_to_matrix(pose):
        # Extract position
        translation = np.array([pose.position.x, pose.position.y, pose.position.z])
        # Extract rotation
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = LidarServiceHandler.quaternion_to_rotation_matrix(q)
        # Create 4x4 homogeneous transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation_matrix
        matrix[:3, 3] = translation
        return matrix

    def publish_state(self, state):
        msg = Bool()
        msg.data = state
        self.state_publisher.publish(msg)
        rospy.loginfo(f"Published lidar state: {state}")

    def lidar_on(self):
        self.publish_state(True)

    def lidar_off(self):
        self.publish_state(False)

    @classmethod
    def get_instance(cls):
        return cls()

if __name__ == "__main__":
    try:
        handler = LidarServiceHandler()

        # Example usage of call_process_pointclouds
        min_z = 0.07
        max_z = 0.08
        cluster_eps = 0.02
        enable_plotting_data = False
        save_figure_to_bitmap_headless = True
        purge_after_processing = False
        handler.call_process_pointclouds(min_z, max_z, cluster_eps, enable_plotting_data, save_figure_to_bitmap_headless, purge_after_processing)

        # Example usage of call_find_line_service
        line_id = 0
        min_z = 0.07
        max_z = 0.08
        # TODO: use with ros pose instead
        # result_matrix = handler.call_find_line_service(line_id, min_z, max_z)
        # if result_matrix is not None:
            # print("Resulting 4x4 Homogeneous Transformation Matrix:")
            # print(result_matrix)

        # Example usage of lidar state control
        handler.lidar_on()
        rospy.sleep(1)  # Wait for 1 second
        handler.lidar_off()

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
