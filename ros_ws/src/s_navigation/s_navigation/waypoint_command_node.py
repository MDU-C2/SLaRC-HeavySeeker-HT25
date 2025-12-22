#!/usr/bin/python3
# Bridge between ROS2 and GPS Waypoint Follower.
# The node takes in wgs84 coordinates and sends them to nav2 gps waypoint follower.
# Example code from navigation 2's tutorial
# https://github.com/ros-navigation/navigation2_tutorials.git

import rclpy
import math
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped
from hs_msgs.msg import WaypointCommandMsgs
from hs_msgs.srv import WaypointCommand
from std_msgs.msg import Bool


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose


class InteractiveGpsWpCommander(Node):
    """
    ROS2 node to send gps waypoints to nav2 received from mapviz's point click publisher
    """

    def __init__(self):
        super().__init__(node_name="gps_wp_commander")
        self.navigator = BasicNavigator("basic_navigator")

        self.waypoints = []

        self.mapviz_wp_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.mapviz_wp_cb, 1)

        self.waypoint_command_srv = self.create_service(WaypointCommand, "/waypoint_command", self.waypoint_command_cb)

        self.activate_autonom_pub = self.create_publisher(Bool, "/activate_autonomous_drive", 10)

    def waypoint_command_cb(self, request: WaypointCommand.Request, response: WaypointCommand.Response):
        """
        Callback function for waypoint command service.
        """

        match (request.command):
            case WaypointCommandMsgs.START:
                response.success = self.start_navigation()
            case WaypointCommandMsgs.STOP:
                response.success = self.stop_navigation()
            case WaypointCommandMsgs.UNDO:
                response.success = self.clear_last_waypoint()
            case WaypointCommandMsgs.CLEAR_ALL:
                response.success = self.clear_all_waypoints()
            case _:
                response.success = False
                response.message = "Invalid command"

        return response


    def start_navigation(self):
        if len(self.waypoints) <= 0:
            self.get_logger().info("No waypoints to navigate to")
            return False

        self.get_logger().info("Starting autonomous navigation")
        self.activate_autonom_pub.publish(True)

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        self.navigator.followGpsWaypoints(self.waypoints)

        return True


    def stop_navigation(self):
        self.get_logger().info("Stopping autonomous navigation")

        self.activate_autonom_pub.publish(False)
        self.navigator.cancelTask()

        return True


    def clear_last_waypoint(self):
        if len(self.waypoints) > 0:
            self.waypoints.pop()
            self.get_logger().info("Cleared last waypoint")
            return True

        return False

    def clear_all_waypoints(self):
        self.get_logger().info("Cleared all waypoint")
        self.waypoints = []

        return True

    def _add_waypoint(self, msg):
        if msg.header.frame_id not in ['wgs84', 'map']:
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return

        wp = latLonYaw2Geopose(msg.point.y, msg.point.x)
        self.waypoints.append(wp)


    def mapviz_wp_cb(self, msg: PointStamped):
        """
        clicked point callback, sends received point to nav2 gps waypoint follower if its a geographic point
        """
        if msg.header.frame_id not in ['wgs84', 'map']:
            self.get_logger().warning(
                "Received point from mapviz that ist not in wgs84 frame. This is not a gps point and wont be followed")
            return

        self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wp = [latLonYaw2Geopose(msg.point.y, msg.point.x)]
        self.navigator.followGpsWaypoints(wp)
        if (self.navigator.isTaskComplete()):
            self.get_logger().info("wps completed successfully")


def main():
    rclpy.init()
    gps_wpf = InteractiveGpsWpCommander()
    rclpy.spin(gps_wpf)


if __name__ == "__main__":
    main()
