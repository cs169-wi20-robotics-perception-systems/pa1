import rospy
import sys

from geometry_msgs.msg import Twist, Point, PoseStamped
from visualization_msgs.msg import Marker

RATE = 10
BUFFER_DISTANCE = 0.01

class Rosbot:
    def __init__(self, distance = 1.0, linear_speed = 0.2):
        self.goal_distance = distance
        self.linear_speed = linear_speed

        self.rate = rospy.Rate(RATE)  # 10 Hz
        self.current_distance = 0.0
        self.poses = []

        rospy.Subscriber("/pose", PoseStamped, self.pose_callback)

        self.command_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.marker_pub = rospy.Publisher("/rosbot8/frame", Marker, queue_size=100)

        rospy.on_shutdown(self.is_shutdown)


    def pose_callback(self, msg):
        """
        Callback for retrieving the robot's position and orientation. Also, adds the current pose to the list of poses for visualization.

        Args:
            msg: Odometry msg that has the current robot's position and orientation.
        """
        current_pose = Point()
        current_pose.x = msg.pose.position.x
        current_pose.y = msg.pose.position.y

        self.poses.append(current_pose)


    def is_shutdown(self):
        """
        Shutdown handler for the robot to ensure that it will stop moving before ending node.
        """
        rospy.loginfo("Rosbot8 shutting down.")

        # Ensure the robot is not moving.
        cmd_msg = Twist()
        self.command_pub.publish(cmd_msg)
        self.rate.sleep()


    def move_forward(self):
        """
        The robot should continously move forward at a set linear velocity until it reaches its goal distance.
        """
        while not rospy.is_shutdown():
            # Robot's distance from its goal.
            distance_to_travel = abs(self.goal_distance - self.current_distance)

            # Robot has not reached target distance and must continue to move forward.
            if BUFFER_DISTANCE < distance_to_travel:
                cmd_msg = Twist()
                cmd_msg.linear.x = self.linear_speed
                self.command_pub.publish(cmd_msg)
                self.rate.sleep()

                # Update current distance of the robot from its starting position.
                self.current_distance = self.current_distance + self.linear_speed / RATE
                rospy.loginfo("Rosbot8 moved %s", self.current_distance)

            # Robot has reached target distance and will proceed to shutdown.
            else:
                rospy.loginfo("Rosbot8 reached approximate target distance of %s.", self.goal_distance)
                rospy.signal_shutdown("Complete.")


    def plot_odom(self):
        """
        Continously plot the robot's x and y pose on a 2D graph.
        """
        while not rospy.is_shutdown():
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time()
            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.points = self.poses
            marker.lifetime = rospy.Duration()
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.a = 1.0

            self.marker_pub.publish(marker)
            self.rate.sleep()
