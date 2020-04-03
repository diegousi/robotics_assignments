#!/usr/bin/env python
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion

from controllers import PID, euclidean_distance


class ThymioController:

    def __init__(self):
        """Initialization."""

        # initialize the node
        rospy.init_node(
            'thymio_controller'  # name of the node
        )

        self.name = rospy.get_param('~robot_name')

        # log robot name to console
        rospy.loginfo('Controlling %s' % self.name)

        # create velocity publisher
        self.velocity_publisher = rospy.Publisher(
            self.name + '/cmd_vel',  # name of the topic
            Twist,  # message type
            queue_size=10  # queue size
        )

        # create pose subscriber
        self.pose_subscriber = rospy.Subscriber(
            self.name + '/odom',  # name of the topic
            Odometry,  # message type
            self.log_odometry  # function that hanldes incoming messages
        )
        self.subscriber_names = ["/proximity/center_right", "/proximity/left",
                              "/proximity/center_left", "/proximity/center",
                              "/proximity/right", "/proximity/rear_left",
                              "/proximity/rear_right"]
        self.sensors_names = ['left', 'center_left', 'center', 'center_right', 
                             'right', 'rear_right', 'rear_left']
        self.sensors_subscribers = {}
        self.sensors_values = {}

        self.init_sensors()


        # tell ros to call stop when the program is terminated
        rospy.on_shutdown(self.stop)

        # initialize pose to (X=0, Y=0, theta=0)
        self.pose = Pose()

        # initialize linear and angular velocities to 0
        self.velocity = Twist()

        # set node update frequency in Hz
        self.rate = rospy.Rate(10)
        self.rate_secs = 1/10

        self.theta = 0
        self.x = 0
        self.y = 0
        self.is_obstacle_infront = False
        self.is_aligned = False
        self.rotating = False
        

    def init_sensors(self):
        for sensor_name in self.subscriber_names:
            key_name = sensor_name.replace("/proximity/", "")

            self.sensors_subscribers[key_name] = rospy.Subscriber(
                self.name + sensor_name, Range, self.log_sensor)
            self.sensors_values[key_name] = {}

    def log_sensor(self, data):
        sensor_name = (data.header.frame_id)
        sensor_name = sensor_name.replace("thymio10/proximity_", "").replace("_link","")
        self.sensors_values[sensor_name] = data.range


    def thym_euc_dist(self, x2, y2):
        return euclidean_distance(self.x, x2, self.y, y2)

    def human_readable_pose2d(self, pose):
        """Converts pose message to a human readable pose tuple."""

        # create a quaternion from the pose
        quaternion = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        # convert quaternion rotation to euler rotation
        roll, pitch, yaw = euler_from_quaternion(quaternion)

        result = (
            pose.position.x,  # x position
            pose.position.y,  # y position
            yaw  # theta angle
        )

        return result

    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist

        printable_pose = self.human_readable_pose2d(self.pose)

        # log robot's pose
        rospy.loginfo_throttle(
            period=5,  # log every 10 seconds
            msg=self.name + ' (%.3f, %.3f, %.3f) ' % printable_pose  # message
        )
        self.theta = printable_pose[2]
        self.x = printable_pose[0]
        self.y = printable_pose[1]

    def angle_difference(self, angle1, angle2=np.pi/2):
        return np.arctan2(np.sin(angle1-angle2), np.cos(angle1-angle2))


    def draw_eight(self, sign, t0, t1):
        tol = 0.1
        if (t1 - t0) > 1 and self.thym_euc_dist(0, 0) <= tol:
            sign = sign * -1
            t0 = rospy.Time.now().to_sec()

        return (sign, t0, Twist(
            linear=Vector3(
                 0.3,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                sign*0.3 #positive values => thym left turn, neg thym right turn.
            )
        )
    )
    def go_straight(self):
        lin_vel = 0.12

        all_sensors = list(self.sensors_values.values())
        #print(all_sensors)
        if any([value <= 0.045 for value in all_sensors]):
            print("went inside!!!")
            # close then stop
            lin_vel = 0.09
            self.is_obstacle_infront = True
    

        return Twist(
            linear=Vector3(
                 lin_vel,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0 
            )
        )

    def align(self, tol=0.9):
        
        front_sensors = [self.sensors_values[sensor_name] for sensor_name in
                        self.sensors_names if 'rear' not in sensor_name]

        are_all_proximal = all([value < 0.05 for value in front_sensors])
        ang_vel = 0.1 if not are_all_proximal else 0.01

        left_turn = (
                (self.sensors_values['left'] < self.sensors_values['right'])
                     or
                (self.sensors_values['center_left'] < self.sensors_values['right'])
                     )
        print("left turn ", left_turn)

        if not left_turn:
            ang_vel *= -1 

        center_sensor = front_sensors.pop(3)
        print("center sensor ", center_sensor)
        print("others ", front_sensors)
        center_condition = (np.isclose(center_sensor, front_sensors[1], tol)
                            and
                            np.isclose(center_sensor, front_sensors[2], tol)
                            ) # center sensor has approx the same as left center
                              # and right center
        print("condition proximity ", are_all_proximal)
        print("condition center ", center_condition)
        if center_condition and are_all_proximal:
            self.is_aligned = True
            print("finished aligningind")
            #self.rotating = True

        return Twist(
            linear=Vector3(
                 0,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                ang_vel 
            )
        )

            

    def rotate(self):
        vel = 0.3
        is_proximal = (self.sensors_values['rear_left'] <= 0.075
                        or
                        self.sensors_values['rear_right'] <= 0.075)
        print("left ", self.sensors_values['rear_left'])
        print("right ", self.sensors_values['rear_right'])
        diff = self.sensors_values['rear_left'] - self.sensors_values['rear_right']
        print("diff {}".format(diff))

        if is_proximal and (np.isclose(diff, 0, 0.005)):

        # if np.isclose(self.theta, np.pi): 
            vel = 0
            self.rotating = False 


        return Twist(
            linear=Vector3(
                .0,  # moves forward .2 m/s
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                vel 
            )
        )

    def run(self):
        """Controls the Thymio."""
        t0 = rospy.Time.now().to_sec()
        turn = 'right'
        sign = 1
        while not rospy.is_shutdown():

            # decide control action
            t1 = rospy.Time.now().to_sec()


            #sign, t0, velocity = self.draw_eight(sign, t0, t1)
            velocity = Twist()
            if not self.is_obstacle_infront:
                velocity = self.go_straight()
            elif not self.is_aligned:
                velocity = self.align()
            else:
                print("stopped")
                #

            # publish velocity message
            self.velocity_publisher.publish(velocity)

            # sleep until next step
            self.rate.sleep()

    def stop(self):
        """Stops the robot."""

        self.velocity_publisher.publish(
            Twist()  # set velocities to 0
        )

        self.rate.sleep()


if __name__ == '__main__':
    controller = ThymioController()

    try:
        controller.run()
    except rospy.ROSInterruptException as e:
        pass
