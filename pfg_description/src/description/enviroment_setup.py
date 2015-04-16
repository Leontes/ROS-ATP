import rospy

from visualization_msgs.msg import Marker

from pfg_tasks import global_vars

def init_environment():
    # Initialize the marker points list.
    waypoint_list = Marker()

    waypoint_list.ns = 'WaypointsNS'

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
    marker_pub = rospy.Publisher('waypointsMarkers', Marker, queue_size = 10) 

    marker_pub.publish(waypoint_list)
    rospy.sleep(1)
    
    for pt in coords:
        waypointText = Marker()

        waypointText.ns = 'WaypointsNS'

        waypointText.id = i

        waypointText.type = Marker.TEXT_VIEW_FACING

        waypointText.action = Marker.ADD

        waypointText.lifetime = rospy.Duration(0)
        
        waypointText.scale.z = 0.25

        waypointText.color.r = 0.5
        waypointText.color.g = 0.7
        waypointText.color.b = 0.2
        waypointText.color.a = 1.0
        
        waypointText.text = pt
        waypointText.pose = coords[pt]

        waypointText.header.frame_id = 'odom'
        waypointText.header.stamp = rospy.Time.now()

        waypointText.pose.position.z += 0.02

        marker_pub.publish(waypointText)
        i = i+1

        waypoint_list.points.append(coords[pt].position)
        

   

    # Publish the waypoint markers
    marker_pub.publish(waypoint_list)