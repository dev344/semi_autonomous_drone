#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz_python_tutorial');
roslib.load_manifest('gazebo')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from gazebo.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import atan, degrees, sqrt

import rospy
from std_msgs.msg import String

RADIAL_CONST = 800

class Controller():
    """ Complex motion controller for the drone. """

    def __init__(self):
        self.actions = {
                'Circle': self.circleObject,
                'Emergency': self.emergency,
                'Takeoff': self.takeoff,
                'Land': self.land,
                'LToR': self.lToR,
                'RToL': self.rToL
        }
        rospy.init_node('drone_controller', anonymous=True)
        rospy.Subscriber("drone_ctrl_directions", String, self.callback)
        self.publisher = rospy.Publisher('cmd_vel', Twist)

    def find_bot_orientation(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        bot_orientation = [-1, -1, -1]
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms('quadrotor', 'world')
            # print resp1.pose
            orientation = resp1.pose.orientation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            bot_orientation = euler_from_quaternion(quaternion)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        return [bot_orientation, gms, resp1]

    def turn_towards_ROI(self, target_x, target_y):
        # Find position of your bot.
        bot_orientation, gms, resp1 = self.find_bot_orientation()

        # Find the direction you are facing
        # print "bot orientation is ", bot_orientation
        yaw = bot_orientation[2]
        if yaw < 0:
            yaw += 6.28

        # Find the direction to turn to
        target_orientation = -1
        if resp1.pose.position.x == target_x:
            if resp1.pose.position.y > target_y:
                target_orientation = 3.14/2.0
            else:
                target_orientation = -3.14/2.0
        else:
            target_orientation = atan( (target_y-resp1.pose.position.y)/(target_x-resp1.pose.position.x) )
            if target_orientation < 0 and (target_y > resp1.pose.position.y):
                target_orientation += 3.14
            elif target_orientation < 0 and (target_y < resp1.pose.position.y):
                target_orientation += 6.28
            elif target_orientation > 0 and (target_y < resp1.pose.position.y):
                target_orientation += 3.14

        # print "Target Details"
        # print target_orientation
        # print degrees(target_orientation)
        # print 'my yaw = ', yaw

        # Turn towards the object.
        while abs(yaw - target_orientation) > 0.1:
            twist = Twist()
            factor1 = -1 if abs(yaw - target_orientation) > 3.14 else 1
            factor2 = -1 if yaw > target_orientation else 1

            factor2 *= factor1

            twist.angular.z = 1.2 * factor2
            self.publisher.publish(twist)

            rospy.sleep(0.04)

            twist.angular.z = 0.0
            self.publisher.publish(twist)
            try:
                resp1 = gms('quadrotor', 'world')
                orientation = resp1.pose.orientation
                quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
                bot_orientation = euler_from_quaternion(quaternion)
                yaw = bot_orientation[2]
                if yaw < 0:
                    yaw += 6.28
                # print "Yaw is", yaw, "and target is", target_orientation
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

    def emergency(self, args):
        pass

    def takeoff(self, args):
        pass

    def land(self, args):
        pass

    def lToR(self, args):
        target_x = float(args[0])
        target_y = float(args[1])

        # Depending on distance from target, move circularly for 
        # appropriate time.
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Taking a circle of radius", radius
        for i in xrange(int(RADIAL_CONST*radius/4)):
            self.turn_towards_ROI(target_x, target_y)
            twist = Twist()
            twist.linear.y = -1.5

            # NOTE: This value must depend on the radius of
            # the circle.
            twist.linear.x = 0.33
            self.publisher.publish(twist)
            rospy.sleep(0.004)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Final radius", radius

    def rToL(self, args):
        target_x = float(args[0])
        target_y = float(args[1])

        # Depending on distance from target, move circularly for 
        # appropriate time.
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Taking a circle of radius", radius
        for i in xrange(int(RADIAL_CONST*radius/4)):
            self.turn_towards_ROI(target_x, target_y)
            twist = Twist()
            twist.linear.y = 1.5

            # NOTE: This value must depend on the radius of
            # the circle.
            twist.linear.x = 0.33
            self.publisher.publish(twist)
            rospy.sleep(0.004)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Final radius", radius

    def circleObject(self, args):
        target_x = float(args[0])
        target_y = float(args[1])

        # Depending on distance from target, move circularly for 
        # appropriate time.
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Taking a circle of radius", radius
        for i in xrange(int(RADIAL_CONST*radius)):
            self.turn_towards_ROI(target_x, target_y)
            twist = Twist()
            twist.linear.y = 1.5

            # NOTE: This value must depend on the radius of
            # the circle.
            twist.linear.x = 0.33
            self.publisher.publish(twist)
            rospy.sleep(0.004)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

        bot_orientation, gms, resp1 = self.find_bot_orientation()
        radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Final radius", radius

    def callback(self, data):
        print rospy.get_name() + ": I heard %s" % data.data
        ctrl_command = data.data.split()
        self.actions[ctrl_command[0]](ctrl_command[1:])
        return
        target_x = float(ctrl_direction[2])
        target_y = float(ctrl_direction[3])

    def listen(self):
        print "Started listening"
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.listen()
