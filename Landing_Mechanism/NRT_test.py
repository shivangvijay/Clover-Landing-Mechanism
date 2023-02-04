#!/usr/bin/env python3
import rospy
import math
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray

class LandingMechanism:
    """ 
    Precise Landing Mechanism for drone, 
    This method can able to handle different wind conditions,
    Tolerances can update accordingly.
    """
    
    def __init__(self):
        """
        Constructor of LandingMechanism class
        """
        self.get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
        self.navigate = rospy.ServiceProxy('navigate', srv.Navigate)
        self.land = rospy.ServiceProxy('land', Trigger)
        self.__x_tolerance = 0.05
        self.__y_tolerance = 0.05
        self.__z_tolerance = 0.35  # this number must be greater than self.height_for_landing
        
        # Height for landing should be as minimum as possible, so the wind conditions won't affect much landing purpose
        self.__height_for_landing = 0.3 

        # Create a Subscription object. Each time a message is posted in aruco_detect/markers, the markers_callback function is called with this message as its argument.
        rospy.Subscriber('aruco_detect/markers', MarkerArray, self.markers_callback)
        
    def get_x_tolerances(self):
        """
        Getter of x tolerance

        Returns:
            x tolerance
        """
        return self.__x_tolerance
    
    def get_y_tolerances(self):
        """
        Getter of y tolerance

        Returns:
            y tolerance
        """
        return self.__y_tolerance
    
    def get_z_tolerances(self):
        """_
        Getter of z tolerance

        Returns:
            z tolerance
        """
        return self.__z_tolerance
    
    def get_height_for_landing(self):
        return self.__height_for_landing

    def quat_to_yaw(self,Qx,Qy,Qz,Qw):
        """
        Function to convert quaterion angles to Euler's yaw angle

        Args:
            Qx (quaterion): quaterion x
            Qy (quaterion): quaterion y
            Qz (quaterion): quaterion z
            Qw (quaterion): quaterion w

        Returns:
            radian: Euler's yaw
        """
        yaw = math.atan2(2.0*(Qy*Qz + Qw*Qx), Qw*Qw - Qx*Qx - Qy*Qy + Qz*Qz)
        return yaw
        
    def markers_callback(self,msg):
        """
        Precise Landing Method

        Args:
            msg (_type_): Data of aruco markers
        """
        
        x_tolerances = self.get_x_tolerances()
        y_tolerances = self.get_y_tolerances()
        z_tolerances = self.get_z_tolerances()
        height_for_landing = self.get_height_for_landing()
        
        print('Detected marker')    # For debug
        for marker in msg.markers:
            # Extracting pose, both position and orientation
            x_pos = marker.pose.position.x
            y_pos = marker.pose.position.y
            z_pos = marker.pose.position.z
            x_orient = marker.pose.orientation.x
            y_orient = marker.pose.orientation.y
            z_orient = marker.pose.orientation.z
            w_orient = marker.pose.orientation.w
            yaw = self.quat_to_yaw(x_orient,y_orient,z_orient,w_orient)

            # navigate drone above aruco marker to land precisely
            self.navigate(frame_id='aruco_100', x=0, y=0, z=height_for_landing)
            telemetry_aruco = self.get_telemetry(frame_id='aruco_100')

            # Important Info 
            print("Aruco Detection Pose:- ",x_pos,y_pos,z_pos,yaw)
            print("Telemetry:- ", telemetry_aruco.x, telemetry_aruco.y, telemetry_aruco.z,telemetry_aruco.yaw)

            if(abs(x_pos - telemetry_aruco.x) < x_tolerances and abs(y_pos - telemetry_aruco.y) < y_tolerances) and abs(z_pos - telemetry_aruco.z) < z_tolerances:
                res = self.land()   # landing 
                if res.success:
                    print('Drone Landed Successful')
                rospy.signal_shutdown("Landing_Done")   
            
            else:
                print("Waiting for precise landing")

if __name__ == '__main__':
    rospy.init_node('Landing_Mechanism')
    LandingMechanism()
    rospy.spin()
