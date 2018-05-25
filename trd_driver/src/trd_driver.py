#!/usr/bin/python2
import serial
import threading
import time
import struct
import math

import rospy
import tf
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry

lock = threading.Lock()

class TrdSerial(threading.Thread):

    def __init__(self, serialport_name, baudrate):
        threading.Thread.__init__(self)
        self._serialport_name = serialport_name
        self._baudrate = baudrate
        self._conn = serial.Serial(self._serialport_name, self._baudrate, timeout=1)
        self._stop = False

        self._first_time_flag = True
        self._encoders_offset = [0, 0]
        self._encoders = [0, 0]
        self._voltage = 0
        self._currents = [0,0]
        self._error_code = 0

    def reconnect(self, side):
        print('{} {} reconnecting...'.format(side, time.time()))
        while True:
            try:
                self._conn = serial.Serial(self._serialport_name, self._baudrate, timeout=1)
            except:
                time.sleep(1)
                print('{} {} reconnecting...'.format(side, time.time()))
            else:
                print('{} {} reconnected!'.format(side, time.time()))
                break

    def run(self):
        msg = []
        msg_len = 0
        in_msg_flag = False
        while not self._stop:
            r = self.read(1)
            if in_msg_flag:
                if len(msg) > 30:
                    in_msg_flag = False
                    msg[:] = []
                    print('Overflow message: {}, discarded!'.format(msg))
                msg.append(r)
                if len(msg)==2:
                    msg_len = ord(r)
                if len(msg) == msg_len+2:
                    if r=='\r':
                        in_msg_flag = False
                        self.update(msg)
                        msg[:] = []
                    else:
                        print('Invalid message: {}, discarded!'.format(msg))
            elif r=='\xea':
                msg.append(r)
                in_msg_flag = True

    def get_encoders(self):
        return (self._encoders[0], self._encoders[1])

    def set_speed(self, v1, v2):
        cmd = [0xea, 0x05, 0x7e, v1, v2, 0x00, 0x0d]
        self.send(cmd)

    def reset_encoder(self):
        print('Reset encoder')
        cmd = [0xea, 0x03, 0x35, 0x00, 0x0d]
        self.send(cmd)

    def reset_base(self):
        print('Reset base')
        cmd = [0xea, 0x03, 0x50, 0x00, 0x0d]
        self.send(cmd)
        time.sleep(3)

    def enable_timeout(self):
        print('Enable timeout')
        cmd = [0xea, 0x02, 0x39, 0x00, 0x0d]
        self.send(cmd)

    def update(self, msg):
        #print('{} received message: {}'.format(time.time(), [hex(ord(m)) for m in msg]))
        if len(msg) < 4:
            print('Msg length < 4 : {}', len(msg))
        if ord(msg[2]) == 0x7e:
            if len(msg) != 23:
                print('Msg length = {} not correct, expected: 23', len(msg))
            self._voltage = 0.2157*ord(msg[3])
            self._currents = [0.0272*ord(msg[4]), 0.0272*ord(msg[5])]
            self._error_code = ord(msg[6])
            self._encoders[0], = struct.unpack('>i', bytearray(msg[11:15]))
            self._encoders[1], = struct.unpack('>i', bytearray(msg[15:19]))
            if self._first_time_flag:
                self._encoders_offset[0] = self._encoders[0]
                self._encoders_offset[1] = self._encoders[1]
                self._first_time_flag = False
            self._encoders[0] -= self._encoders_offset[0]
            self._encoders[1] -= self._encoders_offset[1]
            #print('{} encoders: {}, voltage: {}V, currents: {}A, {}A. send:{} {}, actual:{} {}'.format(
            #            time.time(), self._encoders, self._voltage, self._currents[0], self._currents[1],
            #            ord(msg[7]), ord(msg[8]), ord(msg[9]), ord(msg[10])))
        else:
            print('{} received message: {}'.format(time.time(), msg))

    def stop(self):
        print('stop')
        self._stop = True
        self._conn.close()


    def send(self, cmd):
        for i in range(len(cmd)-2):
            cmd[-2] ^= cmd[i]
        while True:
            try:
                self._conn.write(cmd)
                break
            except serial.serialutil.SerialException:
                self.reconnect('send')

    def read(self, n):
        r = ''
        try:
            r = self._conn.read(n)
        except:
            self.reconnect('read')
        return r

class RosWraperTrd():

    def __init__(self, node_name):
        rospy.init_node(node_name,disable_signals=True)
        self.serialport_name = rospy.get_param('~serialport_name', \
                                               default='/dev/motor_trd')
        self.baudrate = rospy.get_param('~baudrate', default=38400)
        self.linear_coef = rospy.get_param('~linear_coef', default=82)
        self.angular_coef = rospy.get_param('~angular_coef', default=14.64)
        self.left_coef = rospy.get_param('~left_coef', default=1)
        self.right_coef = rospy.get_param('~right_coef', default =1)
        self.encoder_ticks_per_rev = rospy.get_param('~encoder_ticks_per_rev', \
                                                     default=1600)
        self.base_width = rospy.get_param('~base_width', default=0.39)
        self.wheel_diameter = rospy.get_param('~wheel_diameter', default=0.125)
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.sub_vel = rospy.Subscriber('/cmd_vel', Twist, self.vel_callback, queue_size=1)
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.encoder1_prev = 0
        self.encoder2_prev = 0
        self.time_prev = rospy.Time.now() - rospy.Time.from_sec(1)
        self.x = 0
        self.y = 0
        self.theta = 0
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'base_link'

        self.trd_serial = TrdSerial(self.serialport_name, self.baudrate)
        self.trd_serial.reset_encoder()
        #self.trd_serial.reset_base()
        self.trd_serial.enable_timeout()

        self.trd_serial.start()

        self.vel1 = 0
        self.vel2 = 0
        self.vel_latest_time = 0
        self.vel_timeout = 1

    def vel_callback(self, vel_msg):
        v1 = self.linear_coef * vel_msg.linear.x
        v2 = self.linear_coef * vel_msg.linear.x
        v1 -= self.angular_coef * vel_msg.angular.z
        v2 += self.angular_coef * vel_msg.angular.z
        v1 *= self.left_coef
        v2 *= self.right_coef
        v1 += 128
        v2 += 128
        v1 = int(v1) if v1<255 else 255
        v2 = int(v2) if v2<255 else 255
        v1 = int(v1) if v1>0 else 0
        v2 = int(v2) if v2>0 else 0
        lock.acquire()
        self.vel1 = v1
        self.vel2 = v2
        lock.release()
        self.vel_latest_time = time.time()

    def update(self):
        #set speed
        if time.time()-self.vel_latest_time > self.vel_timeout:
            #print('Vel timeout, set to (0,0)')
            lock.acquire()
            self.vel1 = 128
            self.vel2 = 128
            lock.release()
        self.trd_serial.set_speed(self.vel1, self.vel2)
        #get encoders
        (encoder1, encoder2) = self.trd_serial.get_encoders()
        time_current = rospy.Time.now()
        time_elapsed = (time_current - self.time_prev).to_sec()
        self.time_prev = time_current
        dleft = self.left_coef * math.pi * self.wheel_diameter * \
                (encoder1 - self.encoder1_prev) / self.encoder_ticks_per_rev
        dright = self.right_coef * math.pi * self.wheel_diameter * \
                (encoder2 - self.encoder2_prev) / self.encoder_ticks_per_rev
        self.encoder1_prev = encoder1
        self.encoder2_prev = encoder2
        d = (dleft + dright) / 2
        dtheta = (dright - dleft) / self.base_width
        if d != 0:
            dx = math.cos(dtheta) * d
            dy = math.sin(dtheta) * d
            self.x += dx*math.cos(self.theta)-dy*math.sin(self.theta)
            self.y += dx*math.sin(self.theta)+dy*math.cos(self.theta)
        self.theta += dtheta

        self.odom.header.stamp = time_current
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        q = tf.transformations.quaternion_from_euler(0,0,self.theta)
        self.odom.pose.pose.orientation.x = q[0]
        self.odom.pose.pose.orientation.y = q[1]
        self.odom.pose.pose.orientation.z = q[2]
        self.odom.pose.pose.orientation.w = q[3]
        self.odom.twist.twist.linear.x = d / time_elapsed
        self.odom.twist.twist.angular.z = dtheta / time_elapsed

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.update()
                self.pub_odom.publish(self.odom)
                self.tf_broadcaster.sendTransform(
                    (self.x,self.y,0),
                    tf.transformations.quaternion_from_euler(0, 0, self.theta),
                    rospy.Time.now(),
                    'base_link',
                    'odom')
                rate.sleep()
            except KeyboardInterrupt:
                print('exit.')
                self.trd_serial.stop()
                break

if __name__ == '__main__':
    ros_wrapper_trd = RosWraperTrd('trd_driver')
    ros_wrapper_trd.run()

