import rospy
from geometry_msgs.msg import Twist


class robot_movings:
    def __init__(self):
        # Initialize the node
        rospy.init_node("cmd_vel_publisher", anonymous=True)

        # Create a publisher for the topic /cmd_vel
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Create a Twist message
        self.vel_msg = Twist()
    


    def stop_move(self):        
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # move forward at 0 m/s

        # Set angular velocity (z axis)
        # vel_msg.angular.z = 0.5  # rotate at 0.5 rad/s

        # Publish the velocity message
        self.pub.publish(self.vel_msg)

    def move(self):
        self.vel_msg.linear.x = 0.5

    def rotate_by_90_to_left(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # move forward at 0.5 m/s
        self.vel_msg.angular.z = 3.14  # rotate at 3.14 rad/s

    def rotate_by_90_to_right(self):
        # Set linear velocity (x axis)
        self.vel_msg.linear.x = 0  # move forward at 0.5 m/s
        self.vel_msg.angular.z = -3.14