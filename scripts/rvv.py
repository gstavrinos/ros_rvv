#!/usr/bin/env python
import math
import rospy
import random
import string
import tf2_ros
import numpy as np
from std_srvs.srv import Empty
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped

area_pub = None
fov_msg = Range()
tf2_buffer = None
grid_timestamps = np.array([])
max_time = 0
last_t = -1

# Shortcut of tf's lookup_transform
def lookupTF(target_frame, source_frame):
    return tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1))

# Service callback that clears the grid timestamps to (re-)initialize viewed area
def clearCallback(req):
    global grid_timestamps
    grid_timestamps = np.array([])
    return []

# OccupancyGrid callback
def ogCallback(og):
    global grid_timestamps, last_t
    try:
        msg = OccupancyGrid()
        msg.header.frame_id = og.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.info = og.info
        msg.data = og.data[:]
        transform = TransformStamped()
        transform.header = msg.header
        map_origin_rand = "map_origin_"+"".join(random.sample(string.lowercase,5))
        transform.child_frame_id = map_origin_rand
        transform.transform.translation = msg.info.origin.position
        # Invalid quaternion in info.origin.orientation
        #  transform.transform.rotation = msg.info.origin.orientation
        transform.transform.rotation.w = 1
        tf2_buffer.set_transform(transform,"ros_rvv_map_origin_generator")
        #  tf = lookupTF(fov_msg.header.frame_id, msg.header.frame_id)
        #  print tf
        #  print "---"
        tf = lookupTF(fov_msg.header.frame_id, map_origin_rand)
        #  print tf
        #  print "###"
        grid = np.reshape(msg.data, (msg.info.width,msg.info.height))
        if np.shape(grid) != np.shape(grid_timestamps):
            if np.size(grid_timestamps) > 0:
                rospy.logwarn("Re-initializing viewed area due to map shape incompatibility...")
            else:
                rospy.loginfo("Initializing viewed area...")
            grid_timestamps = np.zeros(np.shape(grid))
        if last_t >= 0:
            # TODO calc viewed grid cells and add the time difference into them
            t = rospy.Time.now()
            last_t = t
        msg.data = list(np.ravel(grid_timestamps*grid))
        #  print og.info
        area_pub.publish(msg)
    except Exception as e:
        rospy.logerr(e)

def init():
    global area_pub, tf2_buffer, fov_msg, max_time
    rospy.init_node("ros_rvv")

    # Parameters

    # Range
    rframe = rospy.get_param("/ros_rvv/frame", "camera_frame")
    rt = rospy.get_param("/ros_rvv/radiation_type", 1) # 0 for ultrasound, 1 for IR
    fov = rospy.get_param("/ros_rvv/field_of_view", math.pi/4) # radians
    r = rospy.get_param("/ros_rvv/range", 1) # meters

    # OccupancyGrid
    pa = rospy.get_param("/ros_rvv/publish_area", True)
    mst = rospy.get_param("/ros_rvv/map_sub_topic", "projected_map2")
    mpt = rospy.get_param("/ros_rvv/map_pub_topic", "/ros_rvv/viewed_area")
    fov_topic = rospy.get_param("/ros_rvv/fov_pub_topic", "/ros_rvv/fov")
    max_time = rospy.get_param("/ros_rvv/max_time", 3) # in secs

    fov_pub = rospy.Publisher(fov_topic, Range, queue_size=1)

    # Constant range msg
    fov_msg = Range()
    fov_msg.header.frame_id = rframe
    fov_msg.radiation_type = rt
    fov_msg.field_of_view = fov
    fov_msg.min_range = r
    fov_msg.max_range = r
    fov_msg.range = r

    if pa:
        tf2_buffer = tf2_ros.Buffer()
        rospy.Service("/ros_rvv/clear_viewed_area", Empty, clearCallback)
        listener = tf2_ros.TransformListener(tf2_buffer)
        area_pub = rospy.Publisher(mpt, OccupancyGrid, queue_size=1)
        rospy.Subscriber(mst, OccupancyGrid, ogCallback)

    while not rospy.is_shutdown():
        fov_msg.header.stamp = rospy.Time.now()
        fov_pub.publish(fov_msg)

if __name__ == "__main__":
    init()

