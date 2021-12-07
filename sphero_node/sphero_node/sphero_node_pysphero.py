import logging
import math
import copy
import geometry_msgs.msg

import rclpy
from rclpy.node import Node

from typing import Dict

from pysphero.core import Sphero
from pysphero.device_api.sensor import CoreTime, Quaternion
import pysphero.device_api.sensor as ps_sensor
import pysphero.device_api.user_io as sphero_user_io
from pysphero.driving import StabilizationIndex, Direction

from datetime import datetime
from datetime import timedelta
import threading

from sensor_msgs.msg import Imu

import std_msgs.msg
import std_srvs.srv
import nav_msgs.msg
import geometry_msgs.msg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import tf_transformations

#odom_msg = nav_msgs.msg.Odometry()
#sensor_data = dict()
#lock_sensor_data = threading.Lock()

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

        self.robot = robot
        self.robot.power.wake()
        self.is_connected = True
        self.sensor_data = dict()
        self.lock_sensor_data = threading.Lock()
        self.robot.sensor.set_notify(self.notify_callback, ps_sensor.CoreTime, ps_sensor.Quaternion, ps_sensor.Locator,
                                ps_sensor.Velocity, ps_sensor.Attitude, ps_sensor.Accelerometer, interval=100)

        self.last_cmd_vel_time = datetime.now()
        self.last_diagnostics_time = datetime.now()
        self.cmd_heading = 0
        self.cmd_speed = 0

        #initialize colors
        self.front_color=sphero_user_io.Color(red=0, green=255, blue=0)
        self.back_color=sphero_user_io.Color(red=255, green=0, blue=0)
        self.robot.user_io.set_all_leds_8_bit_mask(front_color=self.front_color, back_color=self.back_color)

        self._init_pubsub()
        self.timer = self.create_timer(0.1, self.pub_odom)



        return
        self._init_params()

        self.update_rate = default_update_rate
        self.sampling_divisor = int(400/self.update_rate)



        self.imu = Imu()
        self.imu.orientation_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.angular_velocity_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.imu.linear_acceleration_covariance = [1e-6, 0., 0., 0., 1e-6, 0., 0., 0., 1e-6]
        self.last_diagnostics_time = datetime.now()
        self.power_state_msg = "No Battery Info"
        self.power_state = 0

    def __del__(self):
        self.stopSphero()

    def stopSphero(self):
        print("stopping")
        #self.robot.set_speed(0)

    def start(self):
        self.timer = self.create_timer(0.1, self.spin)

    def spin(self):
        now=self.get_clock().now()
        if (now-self.last_cmd_vel_time) > self.cmd_vel_timeout:
            # cmd timed out, set to 0
            if self.cmd_heading != 0 or self.cmd_speed != 0:
                self.get_logger().info("setting to 0")
                self.cmd_heading = 0
                self.cmd_speed = 0
                self.robot.driving.drive_with_heading(self.cmd_heading, self.cmd_speed , Direction.forward)
            #if (now-self.last_diagnostics_time) > self.diag_update_rate:
            #    self.last_diagnostics_time = now
            #    self.publish_diagnostics(now)

    def _init_pubsub(self):
        self.odom_pub = self.create_publisher(nav_msgs.msg.Odometry, 'odom', 10)
        #self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        #self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.back_led_sub = self.create_subscription(std_msgs.msg.ColorRGBA, 'set_back_led', self.sub_set_back_led, 10)
        self.front_led_sub = self.create_subscription(std_msgs.msg.ColorRGBA, 'set_front_led', self.sub_set_front_led, 10)
        self.main_led_sub = self.create_subscription(std_msgs.msg.ColorRGBA, 'set_main_led', self.sub_set_main_led, 10)
        self.cmd_vel_sub = self.create_subscription(geometry_msgs.msg.Twist, 'cmd_vel', self.sub_cmd_vel, 10)
        self.stabilization_srv = self.create_service(std_srvs.srv.SetBool, 'set_stabilization', self.srv_set_stabilization)
        self.aim_srv = self.create_service(std_srvs.srv.SetBool, 'reset_aim', self.srv_reset_aim)
        self.heading_sub = self.create_subscription(std_msgs.msg.Float32, 'set_heading', self.set_heading, 10)
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
            color = sphero_user_io.Color(red=red, green=green, blue=blue)
            self.robot.user_io.set_all_leds_8_bit_mask(front_color=self.front_color, back_color=color)

    def sub_set_front_led(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info('Setting front LED to red: {0}, green: {1}, blue: {2}'.format(str(red), str(green), str(blue)))
        if self.is_connected:
            color = sphero_user_io.Color(red=red, green=green, blue=blue)
            self.robot.user_io.set_all_leds_8_bit_mask(front_color=color, back_color=self.back_color)

    def sub_set_main_led(self, msg):
        red = int(msg.r * 255)
        green = int(msg.g * 255)
        blue = int(msg.b * 255)
        self.get_logger().info('Setting main LED to red: {0}, green: {1}, blue: {2}'.format(str(red), str(green), str(blue)))
        if self.is_connected:
            color = sphero_user_io.Color(red=red, green=green, blue=blue)
#            pixel = sphero_user_io.Pixel(x=3, y=4)
            self.robot.user_io.set_led_matrix_one_color(color)

    def sub_cmd_vel(self, msg):

        if self.is_connected:
            self.last_cmd_vel_time = self.get_clock().now()#datetime.now()
            #self.cmd_heading = int(self.normalize_angle_positive(math.atan2(msg.linear.x,msg.linear.y))*180/math.pi)
            #self.cmd_heading = max(0, min(self.cmd_heading, 359))
            self.cmd_heading = 90
            self.cmd_speed = int(math.sqrt(math.pow(msg.linear.x,2)+math.pow(msg.linear.y,2))*255)
            self.cmd_speed = max(-255, min(self.cmd_speed, 255)) #clip speed to range between -255 and 255
            self.get_logger().info('Setting speed: {0}, heading: {1}'.format(str(self.cmd_speed), str(self.cmd_heading)))
            self.cmd_speed = 0
            if self.cmd_speed >= 0:
                self.robot.driving.drive_with_heading(self.cmd_speed, self.cmd_heading, Direction.forward)
            else:
                cmd_speed = math.abs(self.cmd_speed)
                self.robot.driving.drive_with_heading(cmd_speed, self.cmd_heading, Direction.reverse)

    def set_heading(self, msg):
        if self.is_connected:
            self.get_logger().info('Setting heading: {1}'.format(str(self.cmd_heading)))
            #"360 -" is due to the different environment representations
            heading_deg = 360 - int(self.normalize_angle_positive(msg.data)*180.0/math.pi)
            self.robot.driving.drive_with_heading(0, self.cmd_heading, Direction.forward)



    def srv_set_stabilization(self, request, response):
        self.get_logger().info('Incoming request to set stabilization ' + str(request.data))

        if (request.data==True):
            self.robot.driving.set_stabilization(StabilizationIndex.full_control_system)
        elif (request.data==False):
            self.robot.driving.set_stabilization(StabilizationIndex.no_control_system)
        response.success = True
        response.message = str("Stabilize set to " + str(request.data))
        return response


    def srv_reset_aim(self, request, response):
        self.get_logger().info('Reset aim service call received.')
        self.robot.driving.reset_yaw()
        return response

    def pub_odom(self):
        #global sensor_data
        self.lock_sensor_data.acquire()
        tmp_data = copy.deepcopy(self.sensor_data)
        self.lock_sensor_data.release()
        print(tmp_data)
        if (len(tmp_data.keys())>0):
            msg = self.fill_odom_msg(tmp_data)
            print (msg)
            self.odom_pub.publish(msg)


    def fill_odom_msg(self, data: Dict):

        # get current location
        odom_x = data.get(ps_sensor.Locator.y) / 100.0
        odom_y = -1.0 * data.get(ps_sensor.Locator.x) / 100.0  # plus is right, negative is left --> using right handed coordinate system

        # TODO: check correct axis orientation --> yaw most likely other direction
        roll = data.get(ps_sensor.Attitude.roll)
        pitch = data.get(ps_sensor.Attitude.pitch)
        yaw = data.get(ps_sensor.Attitude.yaw)
        q_orientation = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # get velocity
        vel_lin_x = data.get(ps_sensor.Velocity.y) / 100.0
        vel_lin_y = -1.0 * data.get(ps_sensor.Velocity.x) / 100.0

        # get gyroscope, i.e. angular velocity
        #vel_ang_z = data.get(ps_sensor.Gyroscope.z) * math.pi / 180.0
        vel_ang_z = 0.0
        # fill odometry message

        odom = nav_msgs.msg.Odometry(header=std_msgs.msg.Header(frame_id="odom"), child_frame_id='base_footprint')
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        odom.header.stamp.sec = seconds
        odom.header.stamp.nanosec = nanoseconds
        #        odom.pose.pose = Pose(position=Point(x=odom_x, y=odom_y, z=0.0),
        #                              orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))

        quat_x = data.get(ps_sensor.Quaternion.x)
        quat_y = data.get(ps_sensor.Quaternion.y)
        quat_z = data.get(ps_sensor.Quaternion.z)
        quat_w = data.get(ps_sensor.Quaternion.w)

        odom.pose.pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=odom_x,
                                                                                 y=odom_y,
                                                                                 z=0.0),
                                                orientation=geometry_msgs.msg.Quaternion(x=quat_x,
                                                                                         y=quat_y,
                                                                                         z=quat_z,
                                                                                         w=quat_w))

        #odom.pose.pose = geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=odom_x,
        #                                                                         y=odom_y,
        #                                                                         z=0.0),
        #                                        orientation=geometry_msgs.msg.Quaternion(x=q_orientation[0],
        #                                                                                 y=q_orientation[1],
        #                                                                                 z=q_orientation[2],
        #                                                                                 w=q_orientation[3]))

        odom.twist.twist = geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(x=vel_lin_x, y=vel_lin_y, z=0.0),
                                                 angular=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=vel_ang_z))
        odom.pose.covariance = self.ODOM_POSE_COVARIANCE
        odom.twist.covariance = self.ODOM_TWIST_COVARIANCE
        return odom
        # publish the message
        #self.sphero_node.pub_odom(odom)
        logging.info('test')


    def notify_callback(self,data: Dict):
        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Quaternion)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
        print("=" * 60)

        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Locator)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Locator (x, y): {info}")

        print(data.get(ps_sensor.Locator))
        print(data.get(ps_sensor.Locator.y))
        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Velocity)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Velocity (x, y): {info}")

        #save received sensor data to global variable
        #global sensor_data
        self.lock_sensor_data.acquire()
        self.sensor_data = copy.deepcopy(data)
        self.lock_sensor_data.release()


class SensorParse:

    def __init__(self, robot):

        self.robot = robot
        #self.sphero_node = sphero_node
        self.robot.sensor.set_notify(self.notify_callback, ps_sensor.CoreTime, ps_sensor.Quaternion, ps_sensor.Locator,
                                ps_sensor.Velocity, ps_sensor.Attitude, ps_sensor.Accelerometer, interval=100)

    def notify_callback(self,data: Dict):
        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Quaternion)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Quaternion (x, y, z, w): {info}")
        print("=" * 60)

        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Locator)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Locator (x, y): {info}")

        print(data.get(ps_sensor.Locator))
        print(data.get(ps_sensor.Locator.y))
        info = ", ".join("{:1.2f}".format(data.get(param)) for param in ps_sensor.Velocity)
        print(f"[{data.get(CoreTime.core_time):1.2f}] Velocity (x, y): {info}")

        #save received sensor data to global variable
        global sensor_data
        lock_sensor_data.acquire()
        sensor_data = copy.deepcopy(data)
        lock_sensor_data.release()

def main(args=None):

    #get MAC address from command line
    #parser = argparse.ArgumentParser(description='Pass MAC Address')
    #parser.add_argument('-m', '--mac_address',  nargs='?', type=str, const="FA:57:99:71:5A:F5", default="FA:57:99:71:5A:F5", help='The MAC address of the sphero robot.')
    #argparse_args = parser.parse_args()

    mac_address = "FA:57:99:71:5A:F5"
    with Sphero(mac_address=mac_address) as robot:

        rclpy.init()
        sphero_node = SpheroROSDriver(robot)
        rclpy.spin(sphero_node)

        rclpy.shutdown()
        robot.sensor.cancel_notify_sensors()
        robot.power.enter_soft_sleep()

if __name__ == "__main__":
    main()
