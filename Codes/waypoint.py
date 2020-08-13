#! /usr/bin/python2

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped

from controller import DroneController
import utils

class WaypointController:
    """
    Waypoint controller for drone
    """

    def __init__(self, drone, threshold_dist=0.2, hover_time=1, verbose=True):
        self.drone = drone
        self.threshold_dist = threshold_dist
        self.verbose = verbose
        self.hover_time = hover_time

        # Rate has to be greater than 2Hz for px4
        self.rate = rospy.Rate(20)

    def goto(self, point):
        """
        Goes to specified point
        """
        while utils.dist(utils.unwrap_pose(self.drone.pose)[0], point) > self.threshold_dist:
            # Attempt to enter into OFFBOARD mode
            if self.drone.current_state.mode != "OFFBOARD":
                self.drone.set_mode(custom_mode="OFFBOARD")
        
            # Set position to move to
            self.drone.set_pose(position=point)
            self.rate.sleep()
    
    def traverse_path(self, path):
        """
        Traverses path given as list of points
        """
        # Wait till connection established between MAVROS and px4 autopilot
        while not rospy.is_shutdown() and not self.drone.current_state.connected:
            self.rate.sleep()

        # Arms the drone
        self.drone.arm()

        # Need to start streaming setpoints before entering OFFBOARD mode
        self.drone.init_stream(self.rate)

        try:
            for p in path:
                if self.verbose: print("Going to ", p)
                self.goto(p)
                rospy.sleep(self.hover_time)

            print("Traversed path")
            self.drone.land()

        # Land drone if keyboard interrupt is recieved        
        except KeyboardInterrupt:
            print("Keyboard interrupt ...")
            self.drone.land()
        
if __name__ == "__main__":

    rospy.init_node("waypoint_controller")
    
    path = [(0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1), (0, 0, 1)]

    drone = DroneController()
    wc = WaypointController(drone)
    wc.traverse_path(path)
