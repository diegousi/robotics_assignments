#!/usr/bin/env python
import numpy as np
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Vector3
from sensor_msgs.msg import Range
from tf.transformations import euler_from_quaternion
from datetime import datetime

def euclidean_distance(x1, x2, y1, y2):
    return np.sqrt( 
                np.power((x2 - x1), 2) + np.power((y2 - y1), 2)
                )

class ThymioController:

    def __init__(self):
        self.angle = None
        self.action_start_time = None
        self.sign = None
        """Initialization."""

        now = datetime.now().strftime("%H:%M:%S")
        # initialize the node
        rospy.init_node(
            "thymio"+now+"_controller"  # name of the node
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
        self.start_rotation_time = None
        

    def init_sensors(self):
        for sensor_name in self.subscriber_names:
            key_name = sensor_name.replace("/proximity/", "")

            self.sensors_subscribers[key_name] = rospy.Subscriber(
                self.name + sensor_name, Range, self.log_sensor)
            self.sensors_values[key_name] = {}

    def log_sensor(self, data):
        sensor_name = (data.header.frame_id)
        robot_name = self.name.replace("/", "")

        sensor_name = sensor_name.replace(robot_name+"/proximity_", "").replace("_link","")
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
        if any([value <= 0.045 for value in all_sensors]):
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

    def align(self, tol=0.05):
        
        front_sensors = [self.sensors_values[sensor_name] for sensor_name in
                        self.sensors_names if 'rear' not in sensor_name]
        print('sensor_value ', front_sensors)
        
        #if(np.isclose(front_sensors[0], front_sensors[4], tol) and np.isclose(front_sensors[1], front_sensors[3], tol)):
        if(np.abs(front_sensors[0]- front_sensors[4])< tol and np.abs(front_sensors[0]- front_sensors[4])< tol):
            ang_vel = 0
            self.is_aligned = True
            self.is_rotating = True
            self.sign = None
            self.start_rotation_time = rospy.Time.now().to_sec()

        elif (self.sign == None):
            if(np.isclose(front_sensors[3], front_sensors[1], 0.01)):
                self.sign = np.sign(front_sensors[4] - front_sensors[0])
                ang_vel = 10 * (front_sensors[4] - front_sensors[0])
            else:
                self.sign = np.sign(front_sensors[3] - front_sensors[1])
                ang_vel = 10 * (front_sensors[3] - front_sensors[1])
        else:
            if(np.isclose(front_sensors[3], front_sensors[1], 0.01)):
                ang_vel = 10 * np.abs(front_sensors[4] - front_sensors[0]) * self.sign 
            else:
                ang_vel = 10 * np.abs(front_sensors[3] - front_sensors[1]) * self.sign 

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
        print('-' )
        rear_left = self.sensors_values['rear_left']
        rear_right = self.sensors_values['rear_right']

        print('rear right', rear_right)
        print('rear left', rear_left)

        is_proximal = (rear_left < 0.08
                        or
                       rear_right < 0.08)
        print('is is_proximal', is_proximal)
        sensor_differences = rear_left - rear_right
        print('sensor differ' ,sensor_differences)

        ang_vel = 0.3 if not is_proximal else 10 * sensor_differences

        if is_proximal and np.abs(sensor_differences) <=0.04: #(1e-3/2):
            vel = 0
            self.is_rotating = False

        current_time = rospy.Time.now().to_sec()
        if self.start_rotation_time is not None:
            elapsed_time = current_time - self.start_rotation_time
            print("elapsed_time ", elapsed_time)
            if elapsed_time > 15:
                all_sensors = [self.sensors_values[sensor_name] > 0.11 for sensor_name in
                        self.sensors_names]
                if all(all_sensors):
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

    def random_rotation(self):
        if(self.angle == None):
            #generating a random angle between -80 and 80
            random = np.random.randint(-60, 60)
            random = (((random/random)*20) + random)*2*np.pi/360
            self.angle = random
            print("Angolo generato =", random*360/np.pi/2, '  ', random)


        if(self.action_start_time == None):
            self.action_start_time = rospy.Time.now().to_sec()
            print('START, ', self.action_start_time)

        ang_vel = self.angle/3

        current_angle = (rospy.Time.now().to_sec()-self.action_start_time)*abs(ang_vel)
        print('current angle ', current_angle)

        if(current_angle < abs(self.angle)):
            print('setto ang velocity ', ang_vel)
            ang_vel = self.angle/3
        else:
            ang_vel = 0
            self.angle = None
            self.action_start_time = None
            self.is_obstacle_infront = False
            self.is_aligned = False     

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
        t = self.tf_listener_.getLatestCommonTime(self.name+"/base_link", self.name)
        p1 = geometry_msgs.msg.PoseStamped()
        p1.header.frame_id = self.name[1:len(self.name)]#"thymio10"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener_.transformPose(self.name+"/base_link", p1)
        #print "Position of the thymio10 in the robot base:"
        #print p_in_base


    def advance_2mts(self):
        #10s at 0.1 m/s -> 1m
        if(self.action_start_time == None): 
            self.action_start_time = rospy.Time.now().to_sec()
            print('START, ', self.action_start_time)

        elapsed_time = rospy.Time.now().to_sec() - self.action_start_time
        print('elapsed_time, ', elapsed_time)
        lin_vel=0.5
        if(elapsed_time*0.5>2):
            self.action_start_time = -1
            lin_vel = 0 

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
            else:
                velocity = self.random_rotation()
                

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