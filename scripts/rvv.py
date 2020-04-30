#!/usr/bin/env python
import math
import rospy
import struct
import random
import string
import tf2_ros
import numpy as np
import tf2_geometry_msgs
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Range, PointCloud2, PointField
from geometry_msgs.msg import TransformStamped, PoseStamped

area_pub = None
map_sub = None
pc2_area_pub = None
pc2_sub = None
fov_msg = Range()
tf2_buffer = None
grid_timestamps = np.array([])
cloud_timestamps = np.array([])
max_time = 0
last_t = None
last_tc = None
lmap = None
lcloud = None
ml = False
cl = True
default_colour = (0,0,0)
rgb_channeli = 1 # green

# Shortcut of tf's lookup_transform
def lookupTF(target_frame, source_frame):
    return tf2_buffer.lookup_transform(target_frame, source_frame, rospy.Time(), rospy.Duration(1))

# Service callback that clears the timestamps to (re-)initialize viewed area
def clearCallback(req):
    global grid_timestamps, cloud_timestamps, last_t, last_tc
    grid_timestamps = np.array([])
    cloud_timestamps = np.array([])
    last_t = None
    last_tc = None
    return []

# A dummy (just checks for 0s) quaternion checker
def invalidQuaternion(q):
    return q.x == 0 and q.y == 0 and q.z == 0 and q.w ==0

# OccupancyGrid viewed area processing
def handleOccupancyGrid(og):
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
                                grid_timestamps[y,x] = min(grid_timestamps[y,x] + (t-last_t).to_sec()/float(max_time)*100, 100)
                        else:
                            grid_timestamps[y,x] = 100

        last_t = t
        msg.data = list(np.ravel(grid_timestamps))
        area_pub.publish(msg)
    except Exception as e:
        rospy.logerr(e)

def inFOV(x, y):
    return math.sqrt(x*x + y*y) <= fov_msg.range and math.atan2(y,x) >= -fov_msg.field_of_view/2 and math.atan2(y,x) <= fov_msg.field_of_view/2

# PointCloud2 viewed area processing
def handlePointCloud2(cloud):
    global cloud_timestamps, last_tc
    try:
        c = list(pc2.read_points(cloud, field_names=("x", "y", "z")))

        rgbc = [[k[0][0],k[0][1],k[0][2],k[1]] for k in list(zip(c, [0.0 for x in c]))]

        rgbfield = PointField()
        rgbfield.name = "rgb"
        rgbfield.offset = 16
        rgbfield.datatype = 7
        rgbfield.count = 1
        fields = cloud.fields + [rgbfield]

        cloud.header.frame_id = cloud.header.frame_id if cloud.header.frame_id[0] != "/" else cloud.header.frame_id[1:]

        tf = lookupTF(fov_msg.header.frame_id, cloud.header.frame_id)

        if np.shape(cloud_timestamps)[0] != np.shape(c)[0]:
            if np.size(cloud_timestamps) > 0:
                rospy.logwarn("Re-initializing pc2 viewed area due to pointcloud shape incompatibility...")
            else:
                rospy.loginfo("Initializing pc2 viewed area...")
            cloud_timestamps = np.full(np.shape(c)[0], 0.0)
        t = rospy.Time.now()
        if last_tc is not None:
            for i in range(len(rgbc)):
                if cloud_timestamps[i] < 1:
                    p = PoseStamped()
                    p.header.frame_id = fov_msg.header.frame_id
                    p.header.stamp = rospy.Time.now()
                    p.pose.position.x = rgbc[i][0]
                    p.pose.position.y = rgbc[i][1]
                    p.pose.position.z = rgbc[i][2]
                    p.pose.orientation.w = 1

                    pctf = tf2_geometry_msgs.do_transform_pose(p, tf)

                    if inFOV(pctf.pose.position.x, pctf.pose.position.y): 
                        if max_time > 0:
                            cloud_timestamps[i] = min(cloud_timestamps[i] + (t-last_tc).to_sec()/float(max_time), 1)
                        else:
                            cloud_timestamps[i] = 1
                rgb = [0,0,0]
                rgb[rgb_channeli] = int(cloud_timestamps[i] * 255)
                rgb = struct.unpack("f", struct.pack("i",int('%02x%02x%02x' % tuple(rgb),16)))[0]
                rgbc[i][3] = default_colour if cloud_timestamps[i] == 0 else rgb
                # Hacky way to check points due to my faulty GPU
                # Uncomment below lines for debugging using point translation instead of colour
                #  rgbc[i][0] = 10 - cloud_timestamps[i] * 10 + c[i][0]
                #  rgbc[i][1] = 10 - cloud_timestamps[i] * 10 + c[i][1]
            pc2_area_pub.publish(pc2.create_cloud(cloud.header, fields, rgbc))

        last_tc = t
    except Exception as e:
        rospy.logerr(e)

# PointCloud2 callback
def pc2Callback(cloud):
    global lcloud
    if cl:
        lcloud = cloud
        pc2_sub.unregister()
        return
    handlePointCloud2(cloud)

# OccupancyGrid callback
def ogCallback(og):
    global lmap
    if ml:
        lmap = og
        map_sub.unregister()
    handleOccupancyGrid(og)

def init():
    global area_pub, pc2_area_pub, tf2_buffer, map_sub, pc2_sub, fov_msg, max_time, ml, cl, default_colour, rgb_channeli
    rospy.init_node("ros_rvv")

    # Parameters

    # Range
    rframe = rospy.get_param("/ros_rvv/frame", "camera_frame")
    fov_topic = rospy.get_param("/ros_rvv/fov_pub_topic", "/ros_rvv/fov")
    rt = rospy.get_param("/ros_rvv/radiation_type", 1) # 0 for ultrasound, 1 for IR
    fov = rospy.get_param("/ros_rvv/field_of_view", math.pi) # radians
    r = rospy.get_param("/ros_rvv/range", 1) # meters
    rate = rospy.get_param("/ros_rvv/range_rate", 5) # Hz

    # OccupancyGrid
    pa = rospy.get_param("/ros_rvv/publish_area", True)
    mst = rospy.get_param("/ros_rvv/map_sub_topic", "projected_map")
    mpt = rospy.get_param("/ros_rvv/map_pub_topic", "/ros_rvv/viewed_area")
    # Map topic does not change, so subscribe only once
    ml = rospy.get_param("/ros_rvv/latched_map", True)

    # PointCloud2
    pap = rospy.get_param("/ros_rvv/publish_area_as_pc2", True)
    cst = rospy.get_param("/ros_rvv/pc2_sub_topic", "octomap_point_cloud_centers")
    cpt = rospy.get_param("/ros_rvv/pc2_pub_topic", "/ros_rvv/viewed_area_pc2")
    # Map topic does not change, so subscribe only once
    cl = rospy.get_param("/ros_rvv/latched_cloud", True)
    default_colour = rospy.get_param("/ros_rvv/default_cloud_colour", (100,100,100))
    rgb_channel = rospy.get_param("/ros_rvv/cloud_viewed_channel", "g")

    rgb_channeli = 1

    if rgb_channel == "r":
        rgb_channeli = 0
    elif rgb_channel == "b":
        rgb_channeli = 2

    default_colour = struct.unpack("f", struct.pack("i", int("%02x%02x%02x" % default_colour, 16)))[0]

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
    if pa or pap:
        tf2_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf2_buffer)
        rospy.Service("/ros_rvv/clear_viewed_area", Empty, clearCallback)
        if pa:
            area_pub = rospy.Publisher(mpt, OccupancyGrid, queue_size=1)
            map_sub = rospy.Subscriber(mst, OccupancyGrid, ogCallback)
        if pap:
            pc2_area_pub = rospy.Publisher(cpt, PointCloud2, queue_size=1)
            pc2_sub = rospy.Subscriber(cst, PointCloud2, pc2Callback)

    while not rospy.is_shutdown():
        fov_msg.header.stamp = rospy.Time.now()
        fov_pub.publish(fov_msg)
        rospy.sleep(1/rate)
        if ml and lmap is not None:
            handleOccupancyGrid(lmap)
        if cl and lcloud is not None:
            handlePointCloud2(lcloud)


if __name__ == "__main__":
    init()

