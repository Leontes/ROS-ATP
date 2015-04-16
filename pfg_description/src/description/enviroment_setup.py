"""

import rospy

from visualization_msgs.msg import Marker

from pfg_tasks import global_vars

def init_environment():
    # Initialize the marker points list.
    waypoint_list = Marker()
    waypoint_list.ns = 'Waypoints'
    waypoint_list.id = 0
    waypoint_list.type = Marker.CUBE_LIST
    waypoint_list.action = Marker.ADD
    waypoint_list.lifetime = rospy.Duration(0)
    waypoint_list.scale.x = 0.2
    waypoint_list.scale.y = 0.2
    waypoint_list.scale.z = 0.05
    waypoint_list.color.r = 1.0
    waypoint_list.color.g = 0.7
    waypoint_list.color.b = 1.0
    waypoint_list.color.a = 1.0

    waypoint_list.header.frame_id = 'odom'
    waypoint_list.header.stamp = rospy.Time.now()
    waypoint_list.points = list()

    coords = global_vars.black_board.getAllCoords()
    # Set a visualization marker at each waypoint        
    for pt in coords:     
        waypoint_list.points.append(coords[pt].position)

    # Define a marker publisher.
    marker_pub = rospy.Publisher('waypoints', Marker, queue_size = 10)

    # Publish the waypoint markers
    marker_pub.publish(waypoint_list)



"""



import rospy

from visualization_msgs.msg import Marker

from pfg_tasks import global_vars

def init_environment():
    # Initialize the marker points list.
    waypoint_list = Marker()

    waypoint_list.ns = 'Waypoints'

    waypoint_list.id = 0

    waypoint_list.type = Marker.CUBE_LIST

    waypoint_list.action = Marker.ADD

    waypoint_list.lifetime = rospy.Duration(0)

    waypoint_list.scale.x = 0.2
    waypoint_list.scale.y = 0.2

    waypoint_list.color.r = 1.0
    waypoint_list.color.g = 0.7
    waypoint_list.color.b = 1.0
    waypoint_list.color.a = 1.0

    waypoint_list.header.frame_id = 'odom'
    waypoint_list.header.stamp = rospy.Time.now()
    waypoint_list.points = list()

    coords = global_vars.black_board.getAllCoords()
    # Set a visualization marker at each waypoint
    i = 1

    # Define a marker publisher.
    marker_pub = rospy.Publisher('waypoint_list', Marker, queue_size = 10) 
    
    for pt in coords:
        waypointText = Marker()

        waypointText.ns = 'Waypoints'

        waypointText.id = i

        waypointText.type = Marker.TEXT_VIEW_FACING

        waypointText.action = Marker.ADD

        waypointText.lifetime = rospy.Duration(0)
        
        waypointText.scale.z = 2.0

        waypointText.color.r = 0.5
        waypointText.color.g = 0.7
        waypointText.color.b = 0.2
        waypointText.color.a = 1.0
        
        waypointText.text = pt
        waypointText.pose = coords[pt]

        marker_pub.publish(waypointText)
        i = i+1

        waypoint_list.points.append(coords[pt].position)
        

   

    # Publish the waypoint markers
    marker_pub.publish(waypoint_list)

