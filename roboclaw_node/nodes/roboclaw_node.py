#!/usr/bin/env python

from math import pi, cos, sin

import diagnostic_msgs
import diagnostic_updater
from roboclaw_driver.roboclaw_driver import Roboclaw
import rclpy
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry

from rclpy.node import Node
from rclpy.executors import ExternalShutdownException 
from rclpy.callback_groups import ReentrantCallbackGroup

import transforms3d
import time
import sys
import threading
import multiprocessing
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


__author__ = "bwbazemore@uga.edu (Brad Bazemore)"


# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width, _node, cb_group):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.node = _node
        self.odom_pub = self.node.create_publisher(Odometry,'/odom', 10, callback_group = cb_group)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0           # M1=left
        self.last_enc_right = 0          # M2=right
        self.last_enc_time = self.node.get_clock().now()
        

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0
        current_time = self.node.get_clock().now()
        d_time = (current_time.nanoseconds - self.last_enc_time.nanoseconds)/1000000000
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            rclpy.logerr("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            rclpy.logerr("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = transforms3d.euler.euler2quat(0, 0, cur_theta)
        current_time = self.node.get_clock().now()
        br = TransformBroadcaster(self.node)

        tfs = TransformStamped()
        tfs.header.stamp = current_time.to_msg()
        tfs.header.frame_id="odom"
        tfs._child_frame_id = "base_footprint"
        
        tfs.transform.rotation.x = quat[0]
        tfs.transform.rotation.y = quat[1]
        tfs.transform.rotation.z = quat[2]
        tfs.transform.rotation.w = quat[3]

        br.sendTransform(tfs) 

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x = quat[0], y = quat[1], z = quat[2], w = quat[3])


        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance
        self.odom_pub.publish(odom)


class Roboclaw_node(Node):

    def __init__(self):
        super().__init__('roboclaw_node')

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}

        self.get_logger().info('Connecting to roboclaw')
       
        dev_name = self.declare_parameter("~dev", "/dev/ttyACM0").value 
        baud_rate = int(self.declare_parameter("~baud", "38400").value)
        
        self.address = int(self.declare_parameter("~address", "128").value)
        if self.address > 0x87 or self.address < 0x80:
            rclpy.logfatal("Address out of range")
            rclpy.shutdown("Address out of range")

        self.roboclaw = Roboclaw(dev_name, baud_rate)

        # TODO need someway to check if address is correct
        try:
            self.roboclaw.Open()
        except Exception as e:
            rclpy.logfatal("Could not connect to Roboclaw")
            self.get_logger().info(e)
            rclpy.shutdown("Could not connect to Roboclaw")

        try:
            self.roboclaw.Open()
        except Exception as e:
            rclpy.shutdown("Could not connect to Roboclaw")

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(diagnostic_updater.
                         FunctionDiagnosticTask("Vitals", self.check_vitals))
        self.updater.update()

        try:
            version = self.roboclaw.ReadVersion(self.address)
        except Exception as e:
            self.get_logger().warn("Problem getting roboclaw version")
            self.get_logger().info(e)
            pass

        if not version[0]:
            self.get_logger().warn("Could not get version from roboclaw")
        else:
            self.get_logger().info(repr(version[1]))

        self.roboclaw.SpeedM1M2(self.address, 0, 0)
        self.roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = float(self.declare_parameter("~max_speed", "1.0").value) 
        self.TICKS_PER_METER = float(self.declare_parameter("~ticks_per_meter", "2495").value)
        self.BASE_WIDTH = float(self.declare_parameter("~base_width", "0.421").value)
        self.INVERT_MOTOR_DIRECTION = self.declare_parameter("~invert_motor_direction", False).value
        self.FLIP_LEFT_AND_RIGHT_MOTORS = self.declare_parameter("~flip_left_and_right_motors", False).value

        self.cb_group = ReentrantCallbackGroup()
        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH, self, self.cb_group)
        self.last_set_speed_time = self.get_clock().now()

        self.subscription = self.create_subscription(Twist,"/base/cmd_vel", self.cmd_vel_callback, 1, callback_group = self.cb_group)   
        self.subscription

        time.sleep(1)

        self.get_logger().info("dev %s" % dev_name)
        self.get_logger().info("baud %d" % baud_rate)
        self.get_logger().info("address %d" % self.address)
        self.get_logger().info("max_speed %f" % self.MAX_SPEED)
        self.get_logger().info("ticks_per_meter %f" % self.TICKS_PER_METER)
        self.get_logger().info("base_width %f" % self.BASE_WIDTH)
        self.get_logger().info("invert_motor_direction %f" % self.INVERT_MOTOR_DIRECTION)
        self.get_logger().info("flip_left_and_right_motors %f" % self.FLIP_LEFT_AND_RIGHT_MOTORS)

    def run(self):
        self.get_logger().info('Starting motor drive')
        try:
            while rclpy.ok():
                if (self.get_clock().now() - self.last_set_speed_time) > rclpy.time.Duration(seconds=0.3):
                    self.get_logger().info("Did not get command for 1 second, stopping")
                    try:
                        self.roboclaw.ForwardM1(self.address, 0)
                        self.roboclaw.ForwardM2(self.address, 0)
                    except OSError as e:
                        rclpy.logerr("Could not stop")
                        self.get_logger().info(e)

                # TODO need find solution to the OSError11 looks like sync problem with serial
                status_left, enc_left, crc_left = None, None, None
                status_right, enc_right, crc_right = None, None, None

                try:
                    status_left, enc_left, crc_left = self.roboclaw.ReadEncM1(self.address)
                except ValueError:
                    pass
                except OSError as e:
                    self.get_logger().warn("ReadEncM1 OSError: %d", e.errno)
                    self.get_logger().info(e) # rclpy.logdebug(e)

                try:
                    status_right, enc_right, crc_right = self.roboclaw.ReadEncM2(self.address)
                except ValueError:
                    pass
                except OSError as e:
                    self.get_logger().warn("ReadEncM2 OSError: %d", e.errno)
                    self.get_logger().info(e)

                # if (enc1 in locals()) and (enc2 in locals()):
                if self.INVERT_MOTOR_DIRECTION:
                    enc_left = -enc_left
                    enc_right = -enc_right

                if self.FLIP_LEFT_AND_RIGHT_MOTORS:
                    enc_left, enc_right = enc_right, enc_left
                try:
                    self.get_logger().info(" Encoders %d %d" % (enc_left, enc_right))
                    self.encodm.update_publish(enc_left, enc_right)  # update_publish expects enc_left enc_right
                    self.updater.update()
                    
                except:
                    print("problems reading encoders")
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        except ExternalShutdownException:
            pass


    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = self.get_clock().now()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vel_right = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vel_left = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0
        
        left_ticks = int(vel_left * self.TICKS_PER_METER)
        right_ticks = int(vel_right * self.TICKS_PER_METER)  # ticks/s


        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if left_ticks == 0 and right_ticks == 0: 
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
            else:
                self.roboclaw.SpeedM1M2(self.address, left_ticks, right_ticks)
        except OSError as e:
            self.get_logger().warn("SpeedM1M2 OSError: %d", e.errno)  
            self.get_logger().info(e)

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = self.roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().info(e)  # rclpy.logdebug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", str((self.roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10)))
            stat.add("Logic Batt V:", str(float(self.roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10)))
            stat.add("Temp1 C:", str(float(self.roboclaw.ReadTemp(self.address)[1] / 10)))
            stat.add("Temp2 C:", str(float(self.roboclaw.ReadTemp2(self.address)[1] / 10)))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().info(e)
        return stat

    def shutdown(self):
        self.get_logger().info('Shutting down')
        try:
            self.roboclaw.ForwardM1(self.address, 0)
            self.roboclaw.ForwardM2(self.address, 0)
        except OSError:
            rclpy.logerr("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                rclpy.logerr("Could not shutdown motors!!!!")
                self.get_logger().info(e)


def main(args=None):
    rclpy.init(args=args)    

    roboclaw_node = Roboclaw_node()
    rate = roboclaw_node.create_rate(10) # 10 Hz
    
    try: 
        executor = MultiThreadedExecutor()
        executor.add_node(roboclaw_node)

        executor.create_task(roboclaw_node.run)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            roboclaw_node.destroy_node()

    except KeyboardInterrupt:
        sys.exit(1)
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        # Destroy the node explicitly >> (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
        roboclaw_node.shutdown()
                
        roboclaw_node.get_logger().info('Exiting')


if __name__ == '__main__':
    main()
