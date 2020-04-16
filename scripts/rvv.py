#!/usr/bin/env python
import math
import rospy
import random
import string
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from std_srvs.srv import Empty
from sensor_msgs.msg import Range
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped

area_pub = None
fov_msg = Range()
tf2_buffer = None
grid_timestamps = np.array([])
max_time = 0
last_t = None

# Shortcut of tf's lookup_transform
def lookupTF(target_frame, source_frame):
    return tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1))

# Service callback that clears the grid timestamps to (re-)initialize viewed area
def clearCallback(req):
    global grid_timestamps, last_t
    grid_timestamps = np.array([])
    last_t = None
    return []

# A dummy (just checks for 0s) quaternion checker
def invalidQuaternion(q):
    return q.x == 0 and q.y == 0 and q.z == 0 and q.w ==0

# OccupancyGrid callback
def ogCallback(og):
    global grid_timestamps, last_t
    try:
        msg = OccupancyGrid()
        msg.header.frame_id = og.header.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.info = og.info

        transform = TransformStamped()
        transform.header = msg.header
        map_origin_rand = "map_origin_"+"".join(random.sample(string.lowercase,5))
        transform.child_frame_id = map_origin_rand
        transform.transform.translation = msg.info.origin.position

        if invalidQuaternion(msg.info.origin.orientation):
            transform.transform.rotation.w = 1
        else:
            transform.transform.rotation.x = msg.info.origin.orientation.x
            transform.transform.rotation.y = msg.info.origin.orientation.y
            transform.transform.rotation.z = msg.info.origin.orientation.z
            transform.transform.rotation.w = msg.info.origin.orientation.w

        tf2_buffer.set_transform(transform,"ros_rvv_map_origin_generator")

        tf = lookupTF(map_origin_rand, fov_msg.header.frame_id)

        # Maybe TODO
        # Add the real map into the viewed area (but how?)
        # The problem with this is that there are only int values in
        # the range of [-1,100] to visualize both the occupancy grid
        # and the viewed area.
        # For now, I am just visualizing the viewed area. And do nothing
        # with the grid variable apart from comparing shapes
        grid = np.reshape(og.data, (msg.info.height, msg.info.width))

        if np.shape(grid) != np.shape(grid_timestamps):
            if np.size(grid_timestamps) > 0:
                rospy.logwarn("Re-initializing viewed area due to map shape incompatibility...")
            else:
                rospy.loginfo("Initializing viewed area...")
            grid_timestamps = np.full(np.shape(grid),-1)
        t = rospy.Time.now()
        if last_t is not None:
            for yaw in np.arange(-fov_msg.field_of_view/2, fov_msg.field_of_view/2, fov_msg.field_of_view*fov_msg.range*msg.info.resolution):
                for r in np.arange(msg.info.resolution, fov_msg.range, msg.info.resolution):

                    p = PoseStamped()
                    p.header.frame_id = fov_msg.header.frame_id
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = r * math.cos(yaw)
                    p.pose.position.y = r * math.sin(yaw)
                    p.pose.orientation.w = 1

                    maptf = tf2_geometry_msgs.do_transform_pose(p, tf)

                    x = int(maptf.pose.position.x / msg.info.resolution)
                    y = int(maptf.pose.position.y / msg.info.resolution)

                    if x >= 0 and x < msg.info.width and y >= 0 and y < msg.info.height:
                        if max_time > 0:
                            if grid_timestamps[y,x] < 100:
                                grid_timestamps[y,x] = min(grid_timestamps[y,x] +(t-last_t).to_sec()/float(max_time)*100, 100)
                        else:
                            grid_timestamps[y,x] = 100

        last_t = t
        msg.data = list(np.ravel(grid_timestamps))
        area_pub.publish(msg)
    except Exception as e:
        rospy.logerr(e)

def init():
    global area_pub, tf2_buffer, fov_msg, max_time
    rospy.init_node("ros_rvv")

    # Parameters

    # Range
    rframe = rospy.get_param("/ros_rvv/frame", "camera_frame")
    fov_topic = rospy.get_param("/ros_rvv/fov_pub_topic", "/ros_rvv/fov")
    rt = rospy.get_param("/ros_rvv/radiation_type", 1) # 0 for ultrasound, 1 for IR
    fov = rospy.get_param("/ros_rvv/field_of_view", math.pi/4) # radians
    r = rospy.get_param("/ros_rvv/range", 1) # meters
    rate = rospy.get_param("/ros_rvv/range_rate", 5) # Hz

    # OccupancyGrid
    pa = rospy.get_param("/ros_rvv/publish_area", True)
    mst = rospy.get_param("/ros_rvv/map_sub_topic", "projected_map2")
    mpt = rospy.get_param("/ros_rvv/map_pub_topic", "/ros_rvv/viewed_area")
    max_time = rospy.get_param("/ros_rvv/max_time", 10) # in secs, <=0 to disable it

    fov_pub = rospy.Publisher(fov_topic, Range, queue_size=1)

    # Constant range msg
    fov_msg = Range()
    fov_msg.header.frame_id = rframe
    fov_msg.radiation_type = rt
    fov_msg.field_of_view = fov
    fov_msg.min_range = r
    fov_msg.max_range = r
    fov_msg.range = r

    # Initialize only if we need the viewed area
    if pa:
        tf2_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf2_buffer)
        area_pub = rospy.Publisher(mpt, OccupancyGrid, queue_size=1)
        rospy.Subscriber(mst, OccupancyGrid, ogCallback)
        rospy.Service("/ros_rvv/clear_viewed_area", Empty, clearCallback)

    while not rospy.is_shutdown():
        fov_msg.header.stamp = rospy.Time.now()
        fov_pub.publish(fov_msg)
        rospy.sleep(1/rate)

if __name__ == "__main__":
    init()

