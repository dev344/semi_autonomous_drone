#!/usr/bin/env python

# Inherited the code from - 
# https://github.com/mikehamer/ardrone_tutorials

# This controller extends the base DroneVideoDisplay class, adding a keypress handler to enable keyboard control of the drone

# Import the ROS libraries, and load the manifest file which through <depend package=... /> will give us access to the project dependencies
import roslib; roslib.load_manifest('semi_autonomous_drone')
import rospy

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

from std_msgs.msg import String

# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
    PitchForward     = QtCore.Qt.Key.Key_W
    PitchBackward    = QtCore.Qt.Key.Key_S
    RollLeft         = QtCore.Qt.Key.Key_A
    RollRight        = QtCore.Qt.Key.Key_D
    YawLeft          = QtCore.Qt.Key.Key_J
    YawRight         = QtCore.Qt.Key.Key_L
    IncreaseAltitude = QtCore.Qt.Key.Key_I
    DecreaseAltitude = QtCore.Qt.Key.Key_K
    Takeoff          = QtCore.Qt.Key.Key_T
    Land             = QtCore.Qt.Key.Key_G
    Emergency        = QtCore.Qt.Key.Key_Space
    Clear            = QtCore.Qt.Key.Key_C
    Test             = QtCore.Qt.Key.Key_Z


# Our controller definition, note that we extend the DroneVideoDisplay class
class EventListener(DroneVideoDisplay):
    # Gestures
    CIRCLE      = 0
    L_TO_R_SEMI = 1
    R_TO_L_SEMI = 2
    UP_TO_D     = 3
    D_TO_UP     = 4
    L_TO_R      = 5
    R_TO_L      = 6
    
    def __init__(self):
        super(EventListener,self).__init__()
        
        self.pitch = 0
        self.roll = 0
        self.yaw_velocity = 0 
        self.z_velocity = 0
        self.publisher = rospy.Publisher('drone_ctrl_directions', String)
        rospy.Subscriber("interface_directions", String, self.callback)
        self.createButtons()
        self.gesture = -1

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
    def keyPressEvent(self, event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Handle the important cases first!
            if key == KeyMapping.Emergency:
                controller.SendEmergency()
                self.publisher.publish(String("Emergency sent"))
            elif key == KeyMapping.Takeoff:
                controller.SendTakeoff()
                self.publisher.publish(String("Takeoff sent"))
            elif key == KeyMapping.Land:
                controller.SendLand()
                self.publisher.publish(String("Land sent"))
            elif key == KeyMapping.Test:
                self.publisher.publish(String("Test action"))
            elif key == KeyMapping.Clear:
                self.points = []
                self.circles = []
            else:
                # Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
                if key == KeyMapping.YawLeft:
                    self.yaw_velocity += 1
                elif key == KeyMapping.YawRight:
                    self.yaw_velocity += -1

                elif key == KeyMapping.PitchForward:
                    self.pitch += 1
                elif key == KeyMapping.PitchBackward:
                    self.pitch += -1

                elif key == KeyMapping.RollLeft:
                    self.roll += 1
                elif key == KeyMapping.RollRight:
                    self.roll += -1

                elif key == KeyMapping.IncreaseAltitude:
                    self.z_velocity += 1
                elif key == KeyMapping.DecreaseAltitude:
                    self.z_velocity += -1

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


    def keyReleaseEvent(self,event):
        key = event.key()

        # If we have constructed the drone controller and the key is not generated from an auto-repeating key
        if controller is not None and not event.isAutoRepeat():
            # Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
            # Now we handle moving, notice that this section is the opposite (-=) of the keypress section
            if key == KeyMapping.YawLeft:
                self.yaw_velocity -= 1
            elif key == KeyMapping.YawRight:
                self.yaw_velocity -= -1

            elif key == KeyMapping.PitchForward:
                self.pitch -= 1
            elif key == KeyMapping.PitchBackward:
                self.pitch -= -1

            elif key == KeyMapping.RollLeft:
                self.roll -= 1
            elif key == KeyMapping.RollRight:
                self.roll -= -1

            elif key == KeyMapping.IncreaseAltitude:
                self.z_velocity -= 1
            elif key == KeyMapping.DecreaseAltitude:
                self.z_velocity -= -1

            # finally we set the command to be sent. The controller handles sending this at regular intervals
            controller.SetCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

    def mousePressEvent(self, event):
        pass

    def image_processing_func(self):
        # Find average point 
        average_point = QtCore.QPoint(0, 0)
        for point in self.points:
            average_point += point
        average_point /= len(self.points)
        print "Average point is", average_point.x(), average_point.y()

        step = self.image.step
        height = self.image.height
        width = self.image.width

        color = [0, 0, 0]
        color[0] = ord(self.image.data[step*average_point.y() + average_point.x()*3])
        color[1] = ord(self.image.data[step*average_point.y() + average_point.x()*3 + 1])
        color[2] = ord(self.image.data[step*average_point.y() + average_point.x()*3 + 2])

        print "color is", color

        average_color = color

        target_x = 0
        target_y = 0
        found_object = 0

        # if(average_color == red):
        #     # go to red object
        # -7.5, 0, 0.63
        if(average_color[0] > 60 and average_color[1] < 30 and average_color[2] < 30):
            print "Found red object"
            target_x = -7.5
            target_y = 0
            found_object = 1

        # if(average_color == green):
        #     # go to green object
        # -6.5, 6, 0.63
        if(average_color[0] < 60 and average_color[1] > 60 and average_color[2] < 30):
            print "Found green object"
            target_x = -6.5
            target_y = 6
            found_object = 1

        #     # go to blue object
        # -3.5, -3, 0.63
        if(average_color[0] == 0 and average_color[1] < 60 and average_color[2] > 60):
            print "Found blue object"
            target_x = -3.5
            target_y = -3
            found_object = 1
        return [found_object, target_x, target_y]

    def circleDrawn(self):
        if abs(self.points[0].x() - self.points[-1].x()) < 30 and \
            abs(self.points[0].y() - self.points[-1].y()) < 30 :
                return True
        else:
            return False

    def lToRSemiCircle(self):
        average_point = QtCore.QPoint(0, 0)
        for point in self.points:
            average_point += point
        average_point /= len(self.points)
        print "Average point is", average_point.x(), average_point.y()
        if self.leftToRightMotion and \
                self.points[-1].y() > average_point.y() + 20:
                    return True
        else:
            return False

    def rToLSemiCircle(self):
        average_point = QtCore.QPoint(0, 0)
        for point in self.points:
            average_point += point
        average_point /= len(self.points)
        print "Average point is", average_point.x(), average_point.y()
        if self.rightToLeftMotion and \
                self.points[-1].y() > average_point.y() + 20:
                    return True
        else:
            return False

    def leftToRightMotion(self):
        if self.points[0].x() < self.points[-1].x() and \
            abs(self.points[0].y() - self.points[-1].y()) < 30 :
                return True
        else:
            return False

    def rightToLeftMotion(self):
        if self.points[0].x() > self.points[-1].x() and \
            abs(self.points[0].y() - self.points[-1].y()) < 30 :
                return True
        else:
            return False

    def downToUpMotion(self):
        if self.points[0].y() < self.points[-1].y() and \
            abs(self.points[0].x() - self.points[-1].x()) < 30 :
                return True
        else:
            return False

    def upToDownMotion(self):
        if self.points[0].y() > self.points[-1].y() and \
            abs(self.points[0].x() - self.points[-1].x()) < 30 :
                return True
        else:
            return False

    def parseGesture(self):
        if self.circleDrawn():
            self.gesture = self.CIRCLE
        elif self.points[0].x() < self.points[-1].x() and \
            abs(self.points[0].y() - self.points[-1].y()) < 30 :
            self.gesture = self.L_TO_R
        elif self.points[0].x() > self.points[-1].x() and \
            abs(self.points[0].y() - self.points[-1].y()) < 30 :
            self.gesture = self.R_TO_L
        elif self.points[0].y() > self.points[-1].y() and \
            abs(self.points[0].x() - self.points[-1].x()) < 30 :
            self.gesture = self.D_TO_UP
        elif self.points[0].y() < self.points[-1].y() and \
            abs(self.points[0].x() - self.points[-1].x()) < 30 :
            self.gesture = self.UP_TO_D

        average_point = QtCore.QPoint(0, 0)
        for point in self.points:
            average_point += point
        average_point /= len(self.points)
        print "Average point is", average_point.x(), average_point.y()
        if self.points[-1].y() < average_point.y() - 20:
            if self.gesture == self.R_TO_L:
                self.gesture = self.R_TO_L_SEMI
            elif self.gesture == self.L_TO_R:
                self.gesture = self.L_TO_R_SEMI

    def mouseReleaseEvent(self, event):
        clear_pane = True
        if len(self.points) > 50:
            self.parseGesture()
            if self.gesture == self.D_TO_UP:
                print "D_TO_UP"
                self.publisher.publish(String("D_TO_UP"))
            elif self.gesture == self.UP_TO_D:
                self.publisher.publish(String("UP_TO_D"))
                print "UP_TO_D"
            elif self.gesture == self.L_TO_R:
                self.publisher.publish(String("L_TO_R"))
                print "L_TO_R"
            elif self.gesture == self.R_TO_L:
                self.publisher.publish(String("R_TO_L"))
                print "R_TO_L"
            else:
                self.resetQimages()
                found_object, target_x, target_y = self.image_processing_func()
                #NOTE: Change code such that 
                # one computation is done and bit is set.
                if found_object == 1:
                    clear_pane = False
                    if self.gesture == self.CIRCLE:
                            self.publisher.publish(String("Circle " + \
                                    str(target_x) + " " + str(target_y)))
                    elif self.gesture == self.L_TO_R_SEMI:
                            self.publisher.publish(String("LToR " + \
                                    str(target_x) + " " + str(target_y)))
                            print "Moving LToR"
                    elif self.gesture == self.R_TO_L_SEMI:
                            self.publisher.publish(String("RToL " + \
                                    str(target_x) + " " + str(target_y)))
                            print "Moving RToL"
            self.points = []
        
        if clear_pane:
            if self.centralWidget.clickedLabel != -1:
                print "label selected", self.centralWidget.clickedLabel
                self.publisher.publish(String("Repeat " + \
                    str(self.centralWidget.clickedLabel)))
                self.centralWidget.clickedLabel = -1

    def mouseDoubleClickEvent(self, event):
        print "Yoda"

    def mouseMoveEvent(self, event):
        xpos = event.pos().x()
        ypos = event.pos().y()
        print xpos, ypos
        self.points.append(event.pos())

    def takeoffClicked(self):
        controller.SendTakeoff()
        self.publisher.publish(String("Takeoff sent"))

    def landClicked(self):
        controller.SendLand()
        self.publisher.publish(String("Land sent"))

    def turn_leftPressed(self):
        controller.SetCommand(0, 0, 1, 0)

    def turn_leftReleased(self):
        controller.SetCommand(0, 0, 0, 0)

    def turn_rightPressed(self):
        controller.SetCommand(0, 0, -1, 0)

    def turn_rightReleased(self):
        controller.SetCommand(0, 0, 0, 0)

    def createButtons(self):
        QtGui.QToolTip.setFont(QtGui.QFont('SansSerif', 10))
        
        take_off_btn = QtGui.QPushButton('Takeoff', self)
        take_off_btn.clicked.connect(self.takeoffClicked)
        take_off_btn.setToolTip('Click here to <b>Takeoff</b>')
        take_off_btn.resize(take_off_btn.sizeHint())
        take_off_btn.move(640 + 10, 0)

        land_btn = QtGui.QPushButton('Land', self)
        land_btn.clicked.connect(self.landClicked)
        land_btn.setToolTip('Click here to <b>Land</b>')
        land_btn.resize(land_btn.sizeHint())
        land_btn.move(640 + 6*80 + 10 + 60, 0)

        turn_left = QtGui.QPushButton('Turn Left', self)
        turn_left.pressed.connect(self.turn_leftPressed)
        turn_left.released.connect(self.turn_leftReleased)
        turn_left.setToolTip('Click here to <b>Turn Left</b>')
        turn_left.resize(turn_left.sizeHint())
        turn_left.move(640 + 10, 5*55 + 30)

        turn_right = QtGui.QPushButton('Turn Right', self)
        turn_right.pressed.connect(self.turn_rightPressed)
        turn_right.released.connect(self.turn_rightReleased)
        turn_right.setToolTip('Click here to <b>Turn Right</b>')
        turn_right.resize(turn_right.sizeHint())
        turn_right.move(640 + 6*80 + 10 + 60, 5*55 + 30)

    def callback(self, data):
        print rospy.get_name() + ": I heard %s" % data.data
        ctrl_command = data.data.split()
        temp = {'ToDraw':True, 'image': self.qimage}
        self.qimages[int(ctrl_command[1])]['ToDraw'] = True
        self.qimages[int(ctrl_command[1])]['image'] = self.qimage


# Setup the application
if __name__=='__main__':
    import sys
    # Firstly we setup a ros node, so that we can communicate with the other packages
    rospy.init_node('ardrone_event_listener')

    # Now we construct our Qt Application and associated controllers and windows
    app = QtGui.QApplication(sys.argv)
    controller = BasicDroneController()
    display = EventListener()

    display.show()

    # executes the QT application
    status = app.exec_()

    # and only progresses to here once the application has been shutdown
    rospy.signal_shutdown('Great Flying!')
    sys.exit(status)
