#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion


def euclidean_distance(x1, x2, y1, y2):
    return np.sqrt( 
                np.power((x2 - x1), 2) + np.power((y2 - y1), 2)
                )

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

        self.tf_listener_ = tf.TransformListener()

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
        self.is_rotating = False
        self.finished = True
        

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
        # print("data ")
        # print(data)
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
        if any([value <= 0.038 for value in all_sensors]):
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

        are_all_proximal = all([value < 0.08 for value in front_sensors])
        ang_vel = 0.1 if not are_all_proximal else 0.01

        left_turn = (
                (self.sensors_values['left'] < self.sensors_values['right'])
                     or
                (self.sensors_values['center_left'] < self.sensors_values['right'])
                     )
        

        if not left_turn:
            ang_vel *= -1 

        center_sensor = front_sensors.pop(3)
        #print("center sensor ", center_sensor)
        #print("others ", front_sensors)
        center_condition = (np.isclose(center_sensor, front_sensors[1], tol)
                            and
                            np.isclose(center_sensor, front_sensors[2], tol)
                            ) # center sensor has approx the same as left center
                              # and right center
        #print("condition proximity ", are_all_proximal)
        #print("condition center ", center_condition)
        if center_condition and are_all_proximal:
            self.is_aligned = True
            self.is_rotating = True
            #print("finished aligningind")

        return Twist(
            linear=Vector3(
                 0,
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
        rear_left = self.sensors_values['rear_left']
        rear_right = self.sensors_values['rear_right']

        is_proximal = (rear_left < 0.08
                        or
                       rear_right < 0.08)

        ang_vel = 0.3 if not is_proximal else 0.1

        print("left ", self.sensors_values['rear_left'])
        print("right ", self.sensors_values['rear_right'])
        diff = np.abs(rear_left - rear_right)
        print("diff {}".format(diff))
        print("diff comp {}".format(diff <= 1e-3 *2/3 ))


        if is_proximal and diff <= (1e-3/2):
            vel = 0
            self.is_rotating = False 


        return Twist(
            linear=Vector3(
                .0, 
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                ang_vel 
            )
        )

    def get_target_location(self):
        theta = self.theta
        t = self.tf_listener_.getLatestCommonTime("thymio10/base_link", "/thymio10")
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = "thymio10"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener_.transformPose("thymio10/base_link", p1)
        print "Position of the thymio10 in the robot base:"
        print p_in_base


    def advance_2mts(self):
        lin_vel = 0
        print(" target is ", self.get_target_location())

        # if thym_euc_dist(x2=, y2=) <= 0.3:
        #     lin_vel = 0
        #     self.finished = True

        return Twist(
            linear=Vector3(
                lin_vel, 
                .0,
                .0,
            ),
            angular=Vector3(
                .0,
                .0,
                .0 
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

            #self.get_target_location()

            if not self.is_obstacle_infront:
                velocity = self.go_straight()
            elif not self.is_aligned:
                velocity = self.align()
            elif self.is_rotating:
                velocity = self.rotate()
            # else:
            #     velocity = self.advance_2mts()
                

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
