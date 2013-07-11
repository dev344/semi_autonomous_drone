#!/usr/bin/env python
import roslib; roslib.load_manifest('rviz_python_tutorial');
roslib.load_manifest('gazebo')
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from gazebo.srv import GetModelState
from tf.transformations import euler_from_quaternion
from math import atan, degrees

import rospy
from std_msgs.msg import String

class Controller():
    def __init__(self):
        rospy.init_node('drone_controller', anonymous=True)
        rospy.Subscriber("drone_ctrl_directions", String, self.callback)
        self.publisher = rospy.Publisher('cmd_vel', Twist)

    def find_bot_orientation(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        bot_orientation = [-1, -1, -1]
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1 = gms('quadrotor', 'world')
            print resp1.pose
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

        print "Target Details"
        print target_orientation
        print degrees(target_orientation)
        print 'my yaw = ', yaw

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

    def circle_object(self, target_x, target_y):
        for i in xrange(1500):
            self.turn_towards_ROI(target_x, target_y)
            twist = Twist()
            twist.linear.y = 0.9

            # NOTE: This value must depend on the radius of
            # the circle.
            twist.linear.x = 0.2
            self.publisher.publish(twist)
            rospy.sleep(0.01)
            twist.linear.y = 0.0
            twist.linear.x = 0.0
            self.publisher.publish(twist)


    def callback(self, data):
        print rospy.get_name() + ": I heard %s" % data.data
        ctrl_direction = data.data.split()
        event = ctrl_direction[0]
        found_object = int(ctrl_direction[1])
        target_x = float(ctrl_direction[2])
        target_y = float(ctrl_direction[3])

        if (event == 'MouseButtonDblClick'):
            if found_object == 0:
                twist = Twist()

                twist.linear.x = 0.9
                self.publisher.publish(twist)
            else:
                print "Heading towards object"
                self.turn_towards_ROI(target_x, target_y)
                twist = Twist()

                twist.linear.x = 1.4
                self.publisher.publish(twist)

        elif (event == 'LeftButton'):
            rotating = ctrl_direction[4]
            if rotating == 'True':
                print "Halting"
                twist = Twist()
                twist.linear.z = -1.0
                self.publisher.publish(twist)
                rospy.sleep(0.05)
                twist.linear.z = 0.0
                self.publisher.publish(twist)
            else:
                print "Rotating Left"
                twist = Twist()
                twist.angular.z = 0.4
                self.publisher.publish(twist)

        elif (event == 'RightButton'):
            if found_object:
                # Make a Circle.
                self.circle_object(target_x, target_y)
            else:
                twist = Twist()
                twist.angular.z = -0.4
                self.publisher.publish(twist)

    def listen(self):
        print "Started listening"
        rospy.spin()

if __name__ == '__main__':
    controller = Controller()
    controller.listen()
