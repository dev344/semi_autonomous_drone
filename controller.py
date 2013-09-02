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

RADIAL_CONST = 850

class Controller():
    """ Complex motion controller for the drone. """
    R_TO_L = 1
    L_TO_R = -1

    def __init__(self):
        """A constructor."""
        self.actions = {
                'Circle': self.circle,
                'Emergency': self.emergency,
                'Takeoff': self.passFunc,
                'Land': self.passFunc,
                'D_TO_UP': self.moveForward,
                'UP_TO_D': self.moveBackward,
                'L_TO_R': self.moveLeft,
                'R_TO_L': self.moveRight,
                'LToR': self.lToR,
                'RToL': self.rToL,
                'Test': self.test,
                'Repeat': self.repeat
        }
        self.history = [0, 0, 0]
        rospy.init_node('drone_controller', anonymous=True)
        rospy.Subscriber("drone_ctrl_directions", String, self.callback)

        self.publisher = rospy.Publisher('cmd_vel', Twist)
        self.publisher2 = rospy.Publisher('interface_directions', String)

    def emergency(self, args):
        pass

    def passFunc(self, args):
        pass

    def land(self, args):
        pass

    def test(self, args):
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.y = 1.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.y = 2.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y
        twist = Twist()
        twist.linear.x = 1.00
        self.publisher.publish(twist)
        rospy.sleep(1.0)
        twist.linear.y = 0.0
        twist.linear.x = 0.0
        self.publisher.publish(twist)
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        print resp1.pose.position.x, resp1.pose.position.y

    def precompute(self, args):
        """Common code to initialize motion."""
        target_x = float(args[0])
        target_y = float(args[1])
        self.history[0] = target_x
        self.history[1] = target_y

        # history[2] corresponds to current location
        # to be used later on for repeat.
        self.history[2] = 0
        return [target_x, target_y]

    def moveLinear(self, x_vel, y_vel):
        for i in xrange(100):
            twist = Twist()
            twist.linear.x = x_vel
            twist.linear.y = y_vel
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)

    def moveForward(self, args):
        print 'mf'
        self.moveLinear(1, 0)

    def moveBackward(self, args):
        print 'mb'
        self.moveLinear(-1, 0)

    def moveLeft(self, args):
        print 'ml'
        self.moveLinear(0, 1)

    def moveRight(self, args):
        print 'mr'
        self.moveLinear(0, -1)

    def lToR(self, args):
        """Function to move drone from left to right."""
        target_x, target_y = self.precompute(args)
        param_list = [8, 7, 6, 5]

        self.circleObject(target_x, target_y,
                          param_list, False, -1)
        self.history[2] = 4
        self.publisher2.publish(String("Snap " + str(4)))

    def rToL(self, args):
        target_x, target_y = self.precompute(args)
        param_list = [4, 5, 6, 7]

        self.circleObject(target_x, target_y,
                          param_list, False, 1)
        self.history[2] = 0
        self.publisher2.publish(String("Snap " + str(0)))

    def repeat(self, args):
        print "Position chosen", args[0]
        dest = int(args[0])
        param = dest - self.history[2]
        direction = 1
        if param < 0:
            param += 8
        if param > 4:
            direction = -1
            param = 8 - param
        self.circleObject(self.history[0],
                          self.history[1], 
                          xrange(param),
                          True, direction)
        self.history[2] = dest

    def circle(self, args):
        target_x, target_y = self.precompute(args)
        self.circleObject(target_x, target_y, xrange(7+1), False, 1)

    def circleObject(self, target_x, target_y, param_list, is_repeat,
                     direction):

        # Depending on distance from target, move circularly for 
        # appropriate time.
        bot_orientation, gms, resp1 = self.find_bot_orientation()
        orig_radius = sqrt((target_x - resp1.pose.position.x)**2 +
                           (target_y - resp1.pose.position.y)**2)

        print "Taking a circle of radius", orig_radius
        orig_yaw = bot_orientation[2]

        prev_yaw = orig_yaw
        self.turn_towards_ROI(target_x, target_y)
        rospy.sleep(1.4)
        for i in param_list:
            yaw_diff = 0
            if not is_repeat:
                self.publisher2.publish(String("Snap " + str(i%8)))
            while abs(yaw_diff) < (3.14/4):
                twist = Twist()
                twist.linear.y = direction * 1.35

                # NOTE: This value must depend on the radius of
                # the circle.
                twist.linear.x = 0.285
                self.publisher.publish(twist)
                rospy.sleep(0.004)
                twist.linear.y = 0.0
                twist.linear.x = 0.0
                self.publisher.publish(twist)
                yaw = self.turn_towards_ROI(target_x, target_y)

                yaw_diff = yaw - prev_yaw
                if yaw_diff > 5.3:
                    yaw_diff -= 6.28
                if yaw_diff < -5.3:
                    yaw_diff += 6.28
                
            bot_orientation, gms, resp1 = self.find_bot_orientation()
            radius = sqrt((target_x - resp1.pose.position.x)**2 +
                               (target_y - resp1.pose.position.y)**2)
            yaw = bot_orientation[2]

            prev_yaw = yaw
            print "radius", radius
            print "yaw", yaw

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
        
        return yaw


if __name__ == '__main__':
    controller = Controller()
    controller.listen()
