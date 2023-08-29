#!/usr/bin/env python

import rospy
from can_msgs.msg import Frame
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from math import sin, cos, atan2, tan, radians
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg

class CANListener:
    def __init__(self):

        rospy.init_node('can_listener', anonymous=True)
        self.speed_car = 0.0 
        self.steering_angle_not_norm = 0.0 #From CAN we get -500 to 500
        self.steering_angle_norm = 0.0 # normalize the CAN from -30 to 30
        # Ackermann variables
        self.wheelbase = rospy.get_param("~wheelbase", 1.9)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        
        self.subscriber = rospy.Subscriber("/received_messages", Frame, self.can_callback)
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.compute_odometry)

    def can_callback(self, msg):

        if msg.id == 1285:
            binary_list = [format(ord(byte), '08b') for byte in msg.data]
            combined_binary = binary_list[2] + binary_list[3]
            
            combined_integer = int(combined_binary, 2)

            if combined_integer & (1 << 15):
                combined_integer = combined_integer - (1 << 16)
            
            self.speed_car = combined_integer * 0.001
            
            # rospy.loginfo("Received CAN frame: ID=%s, data=%s, value=%.3f", msg.id, combined_binary, self.speed_car)
        
        if msg.id == 1282:
            binary_list = [format(ord(byte), '08b') for byte in msg.data]
            combined_binary = binary_list[3] + binary_list[4]
            
            combined_integer = int(combined_binary, 2)

            real_value = combined_integer * 1.0  # This is just the integer value itself

            self.steering_angle_not_norm = real_value - 500.0
            self.steering_angle_norm = radians((self.steering_angle_not_norm + 500) * 0.06 - 30)
            
            # rospy.loginfo("Received CAN frame: ID=%s, data=%s, value=%.1f, real_data=%.1f", msg.id, combined_binary, self.steering_angle_norm, self.steering_angle_not_norm)

    def compute_odometry(self, event):
        if not event.current_real or not event.last_real:
                rospy.logwarn("Invalid TimerEvent received. Skipping this cycle.")
                return
        # rospy.logwarn("ODOM BEING PUBLISHING")
        current_time = rospy.Time.now()
        # dt = event.current_real.to_sec() - event.last_real.to_sec()
        dt = 0.1
        delta_x = self.speed_car * cos(self.th) * dt
        delta_y = self.speed_car * sin(self.th) * dt
        delta_th = self.speed_car * tan(self.steering_angle_norm) / self.wheelbase * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        

        self.th = atan2(sin(self.th), cos(self.th))
        
        # Publish the odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = sin(self.th / 2.0)
        odom.pose.pose.orientation.w = cos(self.th / 2.0)
        
        self.odom_pub.publish(odom)
        
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation.z = sin(self.th / 2.0)
        transform.transform.rotation.w = cos(self.th / 2.0)

        self.tf_broadcaster.sendTransform(transform)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    listener = CANListener()
    listener.spin()