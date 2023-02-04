import rospy
from clover import srv
from std_srvs.srv import Trigger
from aruco_pose.msg import MarkerArray
import geometry_msgs


rospy.init_node('flight') # 'flight' is name of your ROS node

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)



telemetry = get_telemetry()
print(telemetry.x, telemetry.y, telemetry.z)
telemetry_aruco = get_telemetry(frame_id='aruco_90')
# telem = get_telemetry(frame_id='aruco_3')
print(telemetry_aruco.x, telemetry_aruco.y, telemetry_aruco.z)



navigate(x=1.5, y=1.5, z=2, speed=0.5, frame_id='body', auto_arm=True)