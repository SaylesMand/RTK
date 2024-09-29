import rospy
from geometry_msgs.msg import Twist

class RobotMovings:
    def __init__(self):
        print("move.init")
        # Initialize the node
        # rospy.init_node("cmd_vel_publisher", anonymous=True)

        # Create a publisher for the topic /cmd_vel
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a Twist message
        self.vel_msg = Twist()

    def stop_move(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # move forward at 0 m/s

        # Set angular velocity (z axis)
        self.vel_msg.angular.z = 0  # stop rotation

        # Publish the velocity message
        self.pub.publish(self.vel_msg)

    def move(self):
        self.vel_msg.linear.x = 20
        self.vel_msg.angular.z = 0  # Ensure no rotation while moving
        self.pub.publish(self.vel_msg)

    def rotate_by_90_to_left(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # stop moving forward
        self.vel_msg.angular.z = 3.14 / 2  # rotate 90 degrees to the left

        # Publish the velocity message
        self.pub.publish(self.vel_msg)

    def rotate_by_90_to_right(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # stop moving forward
        self.vel_msg.angular.z = -3.14 / 2  # rotate 90 degrees to the right

        # Publish the velocity message
        self.pub.publish(self.vel_msg)

    def rotate_by_180_to_left(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # stop moving forward
        self.vel_msg.angular.z = 3.14  # rotate 180 degrees to the left

        # Publish the velocity message
        self.pub.publish(self.vel_msg)


