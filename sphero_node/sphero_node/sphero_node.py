import rclpy
from rclpy.node import Node

from spherov2 import scanner
from spherov2.sphero_edu import SpheroEduAPI
from spherov2.types import Color

from datetime import datetime
from datetime import timedelta
import argparse

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import ColorRGBA, Float32
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

from transformations.transformations import *
from typing import Dict

class SpheroROSDriver(Node):

    battery_state =  {1:"Battery Charging",
                      2:"Battery OK",
                      3:"Battery Low",
                      4:"Battery Critical"}


    ODOM_POSE_COVARIANCE = [1e-3, 0., 0., 0., 0., 0.,
                            0., 1e-3, 0., 0., 0., 0.,
                            0., 0., 1e6, 0., 0., 0.,
                            0., 0., 0., 1e6, 0., 0.,
                            0., 0., 0., 0., 1e6, 0.,
                            0., 0., 0., 0., 0., 1e3]


    ODOM_TWIST_COVARIANCE = [1e-3, 0., 0., 0., 0., 0.,
                             0., 1e-3, 0., 0., 0., 0.,
                             0., 0., 1e6, 0., 0., 0.,
                             0., 0., 0., 1e6, 0., 0.,
                             0., 0., 0., 0., 1e6, 0.,
                             0., 0., 0., 0., 0., 1e3]

    def __init__(self, robot, default_update_rate=50.0):
        super().__init__('SpheroROSDriver')
        self.is_connected = True
        self.robot = robot
        self.robot.set_main_led(Color(r=255, g=0, b=0))

        self._init_pubsub()
        self._init_params()

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)

        self.last_cmd_vel_time = datetime.now()
        self.last_diagnostics_time = datetime.now()
        self.cmd_heading = 0
        self.cmd_speed = 0

        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.last_cmd_vel_time = datetime.now()
        self.last_diagnostics_time = datetime.now()
        self.cmd_heading = 0
        self.cmd_speed = 0
        self.power_state_msg = "No Battery Info"
        self.power_state = 0
        self.spin_duration = 0.1

        self.start()

    def __del__(self):
        self.stopSphero()

    def stopSphero(self):
        self.robot.set_speed(0)

    def start(self):
        self.timer = self.create_timer(self.spin_duration, self.spin)

    def spin(self):

        self.pub_odom()
        now = datetime.now()
        if (now-self.last_cmd_vel_time) > self.cmd_vel_timeout:
            # cmd timed out, set to 0
            if self.cmd_heading != 0 or self.cmd_speed != 0:
                self.get_logger().info("setting to 0")
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.robot.roll(int(self.cmd_speed), int(self.cmd_heading), self.spin_duration)
            if (now-self.last_diagnostics_time) > self.diag_update_rate:
                self.last_diagnostics_time = now
                self.publish_diagnostics(now)

    def _init_pubsub(self):
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        #self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        #self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.back_led_sub = self.create_subscription(ColorRGBA, 'set_back_led', self.sub_set_back_led, 10)
        self.front_led_sub = self.create_subscription(ColorRGBA, 'set_front_led', self.sub_set_front_led, 10)
        self.main_led_sub = self.create_subscription(ColorRGBA, 'set_main_led', self.sub_set_main_led, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.sub_cmd_vel, 10)
        self.stabilization_srv = self.create_service(SetBool, 'set_stabilization', self.srv_set_stabilization)
        self.aim_srv = self.create_service(SetBool, 'reset_aim', self.srv_reset_aim)
        self.heading_sub = self.create_subscription(Float32, 'set_heading', self.set_heading, 10)
#        self.angular_velocity_sub = self.create_subscription(Float32, 'set_angular_velocity', self.set_angular_velocity, 10)

    def _init_params(self):
        self.connect_color_red = 0 #rospy.get_param('~connect_red',0)
        self.connect_color_blue = 0 #rospy.get_param('~connect_blue',0)
        self.connect_color_green = 255 #rospy.get_param('~connect_green',255)
        self.cmd_vel_timeout = timedelta(seconds = 0.6) #rospy.Duration(rospy.get_param('~cmd_vel_timeout', 0.6))
        self.diag_update_rate = timedelta(seconds= 1.0) #rospy.Duration(rospy.get_param('~diag_update_rate', 1.0))

    def normalize_angle_positive(self, angle):
        return math.fmod(math.fmod(angle, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)

    def pub_diagnostics(self, time):
        diag = DiagnosticArray()
        diag.header.stamp.sec = time.second
        diag.header.stamp.nanosec = time.microsecond * 1000

        stat = DiagnosticStatus(name="Battery Status", level=DiagnosticStatus.OK, message=self.power_state_msg)
        if self.power_state == 3:
            stat.level = DiagnosticStatus.WARN
        if self.power_state == 4:
            stat.level = DiagnosticStatus.ERROR
        diag.status.append(stat)
        self.diag_pub.publish(diag)

    def sub_set_back_led(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info('Setting back LED to red: {0}, green: {1}, blue: {2}'.format(str(red), str(green), str(blue)))
        if self.is_connected:
            self.robot.set_back_led(Color(r=red,g=green,b=blue))

    def sub_set_front_led(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info('Setting front LED to red: {0}, green: {1}, blue: {2}'.format(str(red), str(green), str(blue)))
        if self.is_connected:
            self.robot.set_front_led(Color(r=red,g=green,b=blue))

    def sub_set_main_led(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info('Setting main LED to red: {0}, green: {1}, blue: {2}'.format(str(red), str(green), str(blue)))
        if self.is_connected:
            self.robot.set_main_led(Color(r=red,g=green,b=blue))

#    def sub_cmd_vel(self, msg):
#        speed = int(msg.linear.x * 100)
#        heading = (int(msg.angular.z) + 3) * 60
#        self.get_logger().info('speed: {0}, heading: {1}'.format(str(speed), str(heading)))
#        self.robot.roll(heading, speed, self.spin_duration)

    def sub_cmd_vel(self, msg):

        if self.is_connected:
            self.last_cmd_vel_time = datetime.now()
            #Compute heading given the velocities. Atan2 is called as atan2(y,x) but here we have to change the coordinate system
            self.cmd_heading = self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi
            self.cmd_speed = int(math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))*255)
            self.cmd_speed = max(-255, min(self.cmd_speed, 255)) #clip speed to range between -255 and 255
            self.get_logger().info('Setting speed: {0}, heading: {1}'.format(str(self.cmd_speed), str(self.cmd_heading)))
            self.robot.roll(int(self.cmd_heading), int(self.cmd_speed), self.spin_duration)

    def set_heading(self, msg):
        if self.is_connected:
            self.get_logger().info('Setting heading: {1}'.format(str(self.cmd_heading)))
            #"360 -" is due to the different environment representations
            heading_deg = 360 - int(self.normalize_angle_positive(msg.data)*180.0/math.pi)
            self.robot.set_heading(heading_deg, False)

    def pub_odom(self, msg):

        self.odom_pub.publish(msg)

    def srv_set_stabilization(self, request, response):
        self.get_logger().info('Incoming request to set stabilization ' + str(request.data))
        self.robot.set_stabilization(request.data)
        response.success = True
        response.message = str("Stabilize set to " + str(request.data))
        return response


    def srv_reset_aim(self, request, response):
        self.get_logger().info('Reset aim service call received.')
        self.robot.reset_aim()
        return response

def main(args=None):

    #get MAC address from command line
    parser = argparse.ArgumentParser(description='Pass MAC Address')
    parser.add_argument('-m', '--mac_address',  nargs='?', type=str, const="FA:57:99:71:5A:F5", default="FA:57:99:71:5A:F5", help='The MAC address of the sphero robot.')
    argparse_args = parser.parse_args()

    #find all available spheros
    toy_list = scanner.find_toys()
    #select the sphero with the mac address given
    for toy in toy_list:
        if toy.address == argparse_args.mac_address:
            print("Found the correct Sphero!")
            break
    else:
        print("Could not find sphero!")
        exit(-1)

    #robot = SpheroEduAPI(toy)
    with SpheroEduAPI(toy)as robot:
        try:
            rclpy.init(args=args)
            sphero_node = SpheroROSDriver(robot)
            rclpy.spin(sphero_node)
        except KeyboardInterrupt:
            print("Keyboard Interrupt!")

        finally:
            rclpy.shutdown()

if __name__ == "__main__":
    main()
