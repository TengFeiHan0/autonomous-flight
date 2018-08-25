#!/usr/bin/env python

import rospy
import subprocess

from drone_controller    import BasicDroneController
from drone_video_display import DroneVideoDisplay

from geometry_msgs.msg import Twist            # work with poses
from geometry_msgs.msg import Vector3 as Vec
from std_msgs.msg      import String           # serialize clicks
from PySide            import QtCore, QtGui


class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key.Key_E
    PitchBackward    = QtCore.Qt.Key.Key_D
    RollLeft         = QtCore.Qt.Key.Key_S
    RollRight        = QtCore.Qt.Key.Key_F
    YawLeft          = QtCore.Qt.Key.Key_W
    YawRight         = QtCore.Qt.Key.Key_R
    IncreaseAltitude = QtCore.Qt.Key.Key_Q
    DecreaseAltitude = QtCore.Qt.Key.Key_A
    Takeoff          = QtCore.Qt.Key.Key_Y
    Land             = QtCore.Qt.Key.Key_H
    Emergency        = QtCore.Qt.Key.Key_Space
    Measurement      = QtCore.Qt.Key.Key_M
    Independent      = QtCore.Qt.Key.Key_I
    LogPose          = QtCore.Qt.Key.Key_P

settings_path = '/home/cipri/catkin_ws/src/slam_controller/waypointPlanner.yaml'
settings_file = """
pit_ref: [0.0, %.3f, %.3f, %.3f]
roll_ref: 0.0
pit_Kp: 0.25
conPit_Kd: 0.01
roll_Kp: 1
conRoll_Kd: 0.1
threshold: 0.02
yaw_ref: -0.01
yaw_Kp: 1
resetStep: 70
"""


class KeyboardController(DroneVideoDisplay):
    def __init__(self):
        super(KeyboardController,self).__init__()

        self.pitch        = 0
        self.roll         = 0
        self.yaw_velocity = 0
        self.z_velocity   = 0
        self.speed_perc   = 0.5

        self.scale       = None
        self.pose        = None
        self.independent = False
        self.subPose     = rospy.Subscriber('/pose', Twist, self.update_pose)
        self.pubClick    = rospy.Publisher('/clicks', String, queue_size=20)

        self.targets = [Twist(linear=Vec(1.0,0.0,0.0))]
        self.next_target = 0

    # Extend DroneVideoDisplay with keyboard handler
    def keyPressEvent(self, event):
        key = event.key()
        # any interaction disables independent mode
        if self.independent:
            self.independent  = False
            self.yaw_velocity = 0
            self.pitch        = 0
            self.roll         = 0
            self.z_velocity   = 0

        if controller is not None and not event.isAutoRepeat():
            if key == KeyMapping.Emergency:
                controller.SendEmergency()
            elif key == KeyMapping.Takeoff:
                controller.SendTakeoff()
            elif key == KeyMapping.Land:
                controller.SendLand()
            else:
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += self.speed_perc
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -self.speed_perc

                elif key == KeyMapping.PitchForward:
                    self.pitch += self.speed_perc
                elif key == KeyMapping.PitchBackward:
                    self.pitch += -self.speed_perc

                elif key == KeyMapping.RollLeft:
                    self.roll += self.speed_perc
                elif key == KeyMapping.RollRight:
                    self.roll += -self.speed_perc

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += self.speed_perc
                elif key == KeyMapping.DecreaseAltitude:
                    self.z_velocity += -self.speed_perc

                # Now handle interactions with the system
                elif key == KeyMapping.Measurement:
                    self.measure_scale()
                elif key == KeyMapping.Independent:
                    if self.scale == None:
                        rospy.logwarn("Cannot go independent. Please adjust scale first.")
                    else:
                        self.independent = True
                        subprocess.call(["roslaunch", "slam_controller", "pid_control.launch"])
                elif key == KeyMapping.LogPose:
                    rospy.loginfo('current pose:\n' + str(self.pose))

            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


    def keyReleaseEvent(self,event):
        key = event.key()
        if controller is not None and not event.isAutoRepeat():
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= self.speed_perc
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -self.speed_perc

            elif key == KeyMapping.PitchForward:
                self.pitch -= self.speed_perc
            elif key == KeyMapping.PitchBackward:
                self.pitch -= -self.speed_perc

            elif key == KeyMapping.RollLeft:
                self.roll -= self.speed_perc
            elif key == KeyMapping.RollRight:
                self.roll -= -self.speed_perc

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= self.speed_perc
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -self.speed_perc

            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    # Extend DroneVideoDisplay with mouse handler
    def mousePressEvent(self,event):
        xleft, yleft, xright, yright = -1, -1, -1, -1
        if event.button() == QtCore.Qt.MouseButton.LeftButton:
            xleft, yleft = event.x(), event.y()
        elif event.button() == QtCore.Qt.MouseButton.RightButton:
            xright, yright = event.x(), event.y()
        self.pubClick.publish("%d %d %d %d" % (xleft, yleft, xright, yright))

    def update_pose(self, new_pose):
        self.pose = new_pose

    def measure_scale(self):
        if self.pose == None:
            rospy.logwarn("Cannot measure scale; pose doesn't exist.")
            return

        self.scale = 1.0 / self.pose.linear.x
        rospy.loginfo("Measuring from:\n" + str(self.pose) + "\nScale=" + str(self.scale))
        self.write_waypoints()

    def get_command(self, event):
        return
        if not self.independent:
            return
        t = self.targets[self.next_target]
        if t.linear.x >= self.pose.linear.x * self.scale:
            self.pitch = 0.2
        else:
            self.pitch = 0
            controller.SendLand()

        controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def write_waypoints(self):
        rospy.loginfo("Writing settings file...")
        self.scale = 13.2
        x1, x2, x3 = 3.7, 3.7 + 4.4, 3.7 + 4.4 + 4.8
        x1 = x1 / self.scale
        x2 = x2 / self.scale
        x3 = x3 / self.scale

        with open(settings_path, 'w') as settings:
            settings.write(settings_file % (x1, x2, x3))
        rospy.loginfo("Finished writing settings file")


# Setup the application
if __name__=='__main__':
    import sys, signal
    rospy.init_node('indep_controller')

    app        = QtGui.QApplication(sys.argv)
    # pid_ctrl   = PIDController()
    controller = BasicDroneController()
    display    = KeyboardController()
    display.show()

    # setup clean exit on Ctrl+C
    signal.signal(signal.SIGINT, lambda s, f: QtGui.QApplication.quit())

    status = app.exec_()

    rospy.signal_shutdown("Terminating")
    sys.exit(status)
