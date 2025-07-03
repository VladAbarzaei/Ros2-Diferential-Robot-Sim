import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from math import atan2, sqrt, pi

class SCR(Node):
    def __init__(self):
        super().__init__("scr_node")

        # Publishers
        # trimite mesaje de tip 'Twist'
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscribers
        # asculta mesajele de la LaserScan si Pose
        self.laser_subscriber = self.create_subscription(LaserScan, "/laser", self.laser_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, "/pose", self.pose_callback, 10)

        # Parametrii de control
        self.target = {"x": 10.0, "y": 0.0, "theta": -pi / 6}
        self.current_pose = {"x": 0.0, "y": 0.0, "theta": pi / 6}
        self.laser_data = []

        # timp bucla de control
        self.control_timer = self.create_timer(0.1, self.control_loop)


    def laser_callback(self, msg):
        """Callback pentru procesarea datelor de la LiDAR."""
        # converteste datele din LiDAR intr-un array numpy
        self.laser_data = np.array(msg.ranges)
        

    def pose_callback(self, msg):
        """Callback pentru procesarea datelor de pozitie."""
        # convertire din quaternion in yaw (theta)
        self.current_pose["x"] = msg.position.x
        self.current_pose["y"] = msg.position.y
        
        q = msg.orientation
        self.current_pose["theta"] = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2))
    
    def control_loop(self):
        """Bucla de control pentru evitarea obstacolelor."""
        if len(self.laser_data) == 0:
            self.get_logger().warn("No laser data available yet.")
            return

        # calculare diferenta dintre tinta si pozitia curenta
        dx = self.target["x"] - self.current_pose["x"]
        dy = self.target["y"] - self.current_pose["y"]

        # folosind pitagora, calculeza distanta pana la tinta
        distance_to_goal = sqrt(dx ** 2 + dy ** 2)

        # calculare unghi catre tinta
        angle_to_goal = atan2(dy, dx)

        # diferenta dintre unghiul catre tinta si orientarea curenta
        angle_diff = angle_to_goal - self.current_pose["theta"]

        # normalizare diferenta de unghi in intervalul [-pi, pi]
        angle_diff = (angle_diff + pi) % (2 * pi) - pi

        twist_msg = Twist()

        # Zonele de detectie a obstacolelor
        min_distance = min(self.laser_data)
        left_zone = np.mean(self.laser_data[:len(self.laser_data) // 3])
        center_zone = np.mean(self.laser_data[len(self.laser_data) // 3:2 * len(self.laser_data) // 3])
        right_zone = np.mean(self.laser_data[2 * len(self.laser_data) // 3:])

        # Logica de a evita obstacolele
        if min_distance < 0.5:  # obstacol detectat in mai putin de 0.5 m
            self.get_logger().info(
                f"Obstacle detected: left={left_zone:.2f}, center={center_zone:.2f}, right={right_zone:.2f}")

            if left_zone > right_zone:
                twist_msg.angular.z = 0.5  # turn left
            else:
                twist_msg.angular.z = -0.5  # turn right

            twist_msg.linear.x = 0.1  # slow down

        elif distance_to_goal > 0.1: # daca tinta nu a fost atinsa
            # navigheaza catre tinta
            if abs(angle_diff) > 0.1:
                twist_msg.angular.z = angle_diff # ajusteaza orientarea robotului
                twist_msg.linear.x = 0.0
            else:
                twist_msg.linear.x = 0.2 # deplasare robot
                twist_msg.angular.z = 0.0
        else: # tinta a fost atinsa
            self.get_logger().info("Goal reached!")
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            return

        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args) # initializare ROS2
    scr_node = SCR() # creare instanta nod 'SCR'
    rclpy.spin(scr_node) # rulare nod pana la oprire manuala
    scr_node.destroy_node() # distrugere nod
    rclpy.shutdown() #inchide ROS2


if __name__ == "__main__":
    main()

