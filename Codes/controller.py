from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode 

class DroneController:
    """
    Simple drone controller class
    """
    def __init__(self):

        # To keep track of current state and pose
        self.current_state = State()
        self.pose = PoseStamped()

        # Subscribe to current state of the drone
        self.state_sub = rospy.Subscriber("mavros/state", State, callback=self.state_cb)
        
        # Subscribe to current position of the drone
        self.pose_sub = rospy.Subscriber("mavros/local_position/pose", PoseStamped, callback=self.pose_cb)

        # Publishisher to set local position to move to
        self.local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
        
        # Service client for arming the drone
        self.arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        
        # Service client for setting the mode of the drone
        self.set_mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)

    def state_cb(self, data):
        """
        Callback for state subscriber. Recieves and stores current state.
        """
        self.current_state = data

    def pose_cb(self, data):
        """
        Callback for pose subscriber. Recieves and stores current pose.
        """
        self.pose = data

    def init_stream(self, rate, n=100):
        """
        Stream setpoint for specified number of times.
        Required before setting mode to OFFBOARD.
        """
        for _ in range(n):
            pose = PoseStamped()
            self.local_pos_pub.publish(pose)
            rate.sleep()

    def arm(self):
        """
        Arms the drone through arming service.
        """
        result = self.arm_service(True)
        print("Arming Drone: ", result)

    def disarm(self):
        """
        Disarms the drone through arming service
        """
        result = self.arm_service(True)
        print("Disarming Drone: ", result)

    def set_mode(self, **kwargs):
        """
        Sets the mode of the drone through set mode service.
        """
        result = self.set_mode_service(**kwargs)
        # print("Setting mode to ", str(kwargs), ": ", result)

    def set_pose(self, position):
        """
        Sets position of drone by publishing local position to move to,
        to the setpoint_position/local topic.
        """
        x, y, z = position

        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        self.local_pos_pub.publish(pose)

    def land(self):
        """
        Lands drone and disarms it
        """
        result = self.set_mode(custom_mode="AUTO.LAND")
        print("Landing drone ")
        if self.pose.pose.position.z < 0.1:
            self.disarm()


if __name__ == "__main__":

    rospy.init_node("move_drone")
    
    # Initialises controller object with its publishers, subscribers and clients
    drone = DroneController()

    # Rate has to be greater than 2Hz for px4
    rate = rospy.Rate(20)

    # Wait till connection established between MAVROS and px4 autopilot
    print("Connecting to autopilot: ", end=' ')
    while not rospy.is_shutdown() and not drone.current_state.connected:
        rate.sleep()
    print("connected")

    # Arms the drone
    drone.arm()

    # Need to start streaming setpoints before entering OFFBOARD mode
    drone.init_stream(rate)

    try:
        # Main loop
        while not rospy.is_shutdown():

            # Attempt to enter into OFFBOARD mode
            if drone.current_state.mode != "OFFBOARD":
                drone.set_mode(custom_mode="OFFBOARD")
            
            # Set position to move to
            drone.set_pose(position=(0, 0, 2))
            rate.sleep()

    # Land drone if keyboard interrupt is recieved        
    except KeyboardInterrupt:
        print(" <-- In keyboard interrupt ...")
        try:
            drone.land()
        except:
            print("Drone landed!")