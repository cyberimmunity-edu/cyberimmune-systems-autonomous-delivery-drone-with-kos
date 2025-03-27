import rclpy
import math
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy

MAX_SPEED = 23.15

def clamp(n, min, max): 
    if n < min: 
        return min
    elif n > max: 
        return max
    else: 
        return n 

def normalize_angle(angle):
    return angle % (2 * math.pi)

def angle_difference(angle1, angle2):
    diff = angle2 - angle1
    return (diff + math.pi) % (2 * math.pi) - math.pi


class MavrosToGazebo(Node):
    def __init__(self):
        super().__init__('mavros_to_gazebo')
        self.rc_subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.mavros_callback,
            10
        )
        
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.imu_subscription = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile
        )
        
        self.imu_model_subscription = self.create_subscription(
            Imu,
            '/model/wildthumper/imu_sensor/imu',
            self.imu_model_callback,
            10
        )
        
        self.front_left_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/front_left_wheel_joint/cmd_vel', 10)
        self.front_right_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/front_right_wheel_joint/cmd_vel', 10)
        self.mid_left_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/mid_left_wheel_joint/cmd_vel', 10)
        self.mid_right_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/mid_right_wheel_joint/cmd_vel', 10)
        self.rear_left_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/rear_left_wheel_joint/cmd_vel', 10)
        self.rear_right_wheel_pub = self.create_publisher(Float64, '/model/wildthumper/joint/rear_right_wheel_joint/cmd_vel', 10)
        
        self.last_mav_yaw = None
        self.last_model_yaw = None
        
    def mavros_callback(self, msg):
        left_velocity = self.rc_to_velocity(msg.channels[0])
        right_velocity = self.rc_to_velocity(msg.channels[2])
        
        if self.last_mav_yaw is not None and self.last_model_yaw is not None:
            yaw_error = angle_difference(self.last_mav_yaw, self.last_model_yaw)
            k_p = 7
            
            max_correction = MAX_SPEED / 2
            correction = -clamp(k_p * yaw_error, -max_correction, max_correction)
            
            left_velocity -= correction
            right_velocity += correction
        
        self.publish_velocity(self.front_left_wheel_pub, left_velocity)
        self.publish_velocity(self.mid_left_wheel_pub, left_velocity)
        self.publish_velocity(self.rear_left_wheel_pub, left_velocity)
        
        self.publish_velocity(self.front_right_wheel_pub, right_velocity)
        self.publish_velocity(self.mid_right_wheel_pub, right_velocity)
        self.publish_velocity(self.rear_right_wheel_pub, right_velocity)

        
    def rc_to_velocity(self, rc_value):
        return (rc_value - 1500) / 500.0 * MAX_SPEED
    
    def publish_velocity(self, publisher, velocity):
        msg = Float64()
        velocity = clamp(velocity, -MAX_SPEED, MAX_SPEED)
        msg.data = velocity
        publisher.publish(msg)
        
    def imu_callback(self, msg):
        orientation_q = msg.orientation
        yaw = self.quaternion_to_yaw(orientation_q)
        self.last_mav_yaw = normalize_angle(yaw)
        
    def imu_model_callback(self, msg):
        orientation_q = msg.orientation
        yaw = self.quaternion_to_yaw(orientation_q)
        self.last_model_yaw = normalize_angle(yaw)
        
    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

def main(args=None):
    rclpy.init(args=args)
    node = MavrosToGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()