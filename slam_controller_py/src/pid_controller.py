#!/usr/bin/env python

import rospy
import tf, math

from geometry_msgs.msg    import Twist
from std_msgs.msg         import Empty
from ardrone_autonomy.msg import Navdata
from drone_status         import DroneStatus

class PIDController(object):
    def __init__(self):
        self.next_point = 1
        self.status = DroneStatus.Unknown

        self.params     = rospy.get_param('/indep_controller')
        self.refs       = self.params['refs']
        self.thres      = self.params['refs']['pitch']['thres']
        self.gains      = self.params['gains']
        self.hover_step = self.params['hover_step']

        self.should_hover = True
        self.finished = False
        self.last_err = {'pitch': 0.0, 'roll': 0.0}
        self.dt = 1.0 / 30.0

        self.pose = Twist()
        self.tf_listener = tf.TransformListener()

        self.subNavdata = rospy.Subscriber('/ardrone/navdata', Navdata, self.receiveNavdata, queue_size = 20)
        self.pubLand    = rospy.Publisher('/ardrone/land', Empty, queue_size = 20)
        self.pubCommand = rospy.Publisher('/cmd_vel'     , Twist, queue_size = 20)
        self.pubPose    = rospy.Publisher('/pose'        , Twist, queue_size = 20)
        self.cmdTimer = rospy.Timer(rospy.Duration(self.dt), self.sendCommand)

    def receiveNavdata(self, navdata):
        self.status = navdata.state

    def calcPose(self):
        pos, rot = None, None
        try:
            now, wait_dur = rospy.Time.now(), rospy.Duration(5.0)
            self.tf_listener.waitForTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera", now, wait_dur)
            (pos, rot) = self.tf_listener.lookupTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera", now)
        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logwarn("Transform not available")

        if pos and rot:
            self.pose.linear.x = pos[2]
            self.pose.linear.y = pos[0] * -1
            self.pose.linear.z = pos[1] * -1

            euler = tf.transformations.euler_from_quaternion(rot)
            self.pose.angular.x = euler[0]
            self.pose.angular.y = euler[1]
            self.pose.angular.z = euler[2]

    def sendCommand(self, event):
        # if self.status != DroneStatus.Flying or self.status != DroneStatus.Hovering:
        #     return
        if self.finished:
            return

        self.calcPose()
        cmd = Twist()

        if self.should_hover:
            if self.hover_step > 0:
                self.hover_step -= 1
            else:
                self.should_hover = False
                self.hover_step = self.params['hover_step']

            # cmd remains empty so it will hover
            return

        pitch_refs = self.refs['pitch']['pos']
        dest_pt    = pitch_refs[self.next_point]
        prev_pt    = pitch_refs[self.next_point - 1]
        pitch_dist = math.fabs(dest_pt - prev_pt)

        yaw_err    = self.refs['yaw']  - self.pose.angular.z
        pitch_err  =    dest_pt        - self.pose.linear.x
        roll_err   = self.refs['roll'] - self.pose.linear.y

        pitch_err_dot = pitch_err - self.last_err['pitch']
        roll_err_dot  = roll_err  - self.last_err['roll']

        cmd.linear.x = self.gains['pitch']['p'] * (pitch_err / pitch_dist) + \
                       self.gains['pitch']['d'] * pitch_err_dot
        cmd.linear.y = self.gains['roll' ]['p'] * roll_err / (3*pitch_dist) + \
                       self.gains['roll' ]['d'] * roll_err_dot

        self.last_err['pitch'] = pitch_err
        self.last_err['roll']  = roll_err

        # keep y within bounds
        if cmd.linear.y < -0.5:
            cmd.linear.y = -0.5
        if cmd.linear.y > 0.5:
            cmd.linear.y = 0.5

        cmd.angular.x = 1.0
        cmd.angular.y = 1.0
        yaw_max = math.pi * 45.0 / 180.0
        cmd.angular.z = -1.0 * self.gains['yaw']['p'] * (yaw_err/yaw_max)

        # are we there yet ?
        if dest_pt - self.thres < cmd.linear.x and cmd.linear.x < dest_pt + self.thres:
            self.should_hover = True
            self.next_point += 1
            if self.next_point == len(pitch_refs):
                # Land
                self.pubLand.publish(Empty())
                self.finished = True

        self.pubCommand.publish(cmd)


if __name__ == "__main__":
    rospy.init_node('pid_controller')
    pid_ctrl   = PIDController()
    while not rospy.is_shutdown():
        rospy.spin()

