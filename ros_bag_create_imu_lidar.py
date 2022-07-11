#!/usr/bin/env python2
#import tf
import os
from pydoc_data.topics import topics
from sqlite3 import Time
#import progressbar
#import cv2
import rospy
import rosbag
#import progressbar
#from tf2_msgs.msg import TFMessage
from datetime import datetime
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
import numpy as np
import argparse
import datetime 
### data format lat:   latitude of the oxts-unit (deg)
rospy.init_node('my_node_name')
'''
0 lat:   latitude of the oxts-unit (deg)
1 lon:   longitude of the oxts-unit (deg)
2 alt:   altitude of the oxts-unit (m)
3 roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
4 pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
5 yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
6 vn:    velocity towards north (m/s)
7 ve:    velocity towards east (m/s)
8 vf:    forward velocity, i.e. parallel to earth-surface (m/s)
9 vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
10 vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
11 ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
12 ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
13 ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
14 af:    forward acceleration (m/s^2)
15 al:    leftward acceleration (m/s^2)
16 au:    upward acceleration (m/s^2)
17 wx:    angular rate around x (rad/s)
18 wy:    angular rate around y (rad/s)
19 wz:    angular rate around z (rad/s)
20 wf:    angular rate around forward axis (rad/s)
21 wl:    angular rate around leftward axis (rad/s)
22 wu:    angular rate around upward axis (rad/s)
pos_accuracy:  velocity accuracy (north/east in m)
vel_accuracy:  velocity accuracy (north/east in m/s)
navstat:       navigation status (see navstat_to_string)
numsats:       number of satellites tracked by primary GPS receiver
posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
'''

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  #print(type(roll), roll)
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


def IMU_bag_create(data, imu_frame_id, topic, time_stamp):
  # 2, 3, 4 rol p y
  q = get_quaternion_from_euler(np.float64(data[3]), np.float64(data[4]), np.float64(data[5]))
  imu = Imu()
  imu.header.frame_id = imu_frame_id
  imu.header.stamp = rospy.Time.from_sec(float(time_stamp)) #rospy.Time.from_sec(float(timestamp.strftime("%s.%f")))
  imu.orientation.x = q[0]
  imu.orientation.y = q[1]
  imu.orientation.z = q[2]
  imu.orientation.w = q[3]
  #wf:    angular rate around forward axis (rad/s) 19
  #wl:    angular rate around leftward axis (rad/s) 20 
  #wu:    angular rate around upward axis (rad/s) 21
  imu.angular_velocity.x = np.float64(data[20])
  imu.angular_velocity.y = np.float64(data[21])
  imu.angular_velocity.z = np.float64(data[22])
  #################################################
  #13 af:    forward acceleration (m/s^2)
  #14 al:    leftward acceleration (m/s^2)
  #15 au:    upward acceleration (m/s^2)
  imu.linear_acceleration.x = np.float64(data[14])
  imu.linear_acceleration.y = np.float64(data[15])
  imu.linear_acceleration.z = np.float64(data[16])
  # angular acce 
  #time_now = velo_datetimes[count]
  #rospy.Time.from_sec(time.time())
  bag.write(topic, imu, t=imu.header.stamp) #t=rospy.Time.now())#


def LiDAR_bag_create(scan, velo_frame_id, topic, time_stamp):
  header = Header()
  header.frame_id = velo_frame_id
  header.stamp = rospy.Time.from_sec(float(time_stamp)) #rospy.Time.from_sec(float(timestamp.strftime("%s.%f"))) #rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))
  # fill pcl msg
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('i', 12, PointField.FLOAT32, 1)]
  pcl_msg = pcl2.create_cloud(header, fields, scan)
  time_now = rospy.Time.now()
  velo_datetimes.append(time_now)

  bag.write(topic, pcl_msg, t=pcl_msg.header.stamp) #t=time_now)#


####
# Read files from Kitti
Main_path = '/home/parees/chao/DeepMappingPP_balance/imu_data/2011_09_30/2011_09_30_drive_0018_sync/oxts/'
path = '/home/parees/chao/DeepMappingPP_balance/imu_data/2011_09_30/2011_09_30_drive_0018_sync/oxts/data/'#os.path.join(Main_path,'data')
velo_path = '/home/parees/chao/DeepMappingPP_balance/lidar_data/dataset/sequences/10/velodyne_points'
velo_dir_path = os.path.join(velo_path, 'data_bin')
velo_data_dir_list = os.listdir(velo_dir_path)
velo_data_dir_list.sort()
#velo_data_dir = os.path.join(velo_path, 'data')
    #velo_filenames = sorted(os.listdir(velo_data_dir))

File_dir_list = os.listdir(path)
File_dir_list.sort()

Time_stamps_list = open(os.path.join(Main_path, 'timestamps.txt'), "r")
Time_stamps_list = Time_stamps_list.readlines()
times_stamps_IMU = []
Velo_time_stamps_list = open(os.path.join(velo_path, 'timestamps.txt'), "r")
Velo_time_stamps_list = Velo_time_stamps_list.readlines()
velo_datetimes = []
#dt = datetime.strptime(line[:-4], '%Y-%m-%d %H:%M:%S.%f')
#velo_datetimes.append(dt)
bag = rosbag.Bag('/home/parees/chao/bag_files/Lidar_IMU_merge_chao.bag', 'w', compression = rosbag.Compression.NONE)


#print("done imu")
#bar = progressbar.ProgressBar()

for i, (velo_file, file) in enumerate(zip(velo_data_dir_list, File_dir_list)):
  if i % 50 == 0:
    print("velo", i)
  #if i <= 600:
  velo_filename = os.path.join(velo_dir_path, velo_file)
  scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)
  velo_topic = "/kitti/velo/pointcloud"
  velo_frame_id = "map"#'/camera_init'#'velo_link' ##'base_link'#'/camera_init'#'velo_link'
  velo_timestamp = Velo_time_stamps_list[i]
  velo_timestamp = velo_timestamp[:26]
  velo_date_time = datetime.datetime.strptime(velo_timestamp, "%Y-%m-%d %H:%M:%S.%f")
  velo_timestamp = velo_date_time.strftime("%s.%f")
  ###### IMU data
  my_file = open(path+file, "r")
  # opening the file in read mode
  # reading the file
  data = my_file.read()
  data = data.split()
  imu_frame_id = "/camera_init"#"map"#'/camera_init'#'imu_link'  #'camera_init'#'imu_base'
  timestamp = Time_stamps_list[i]
  timestamp = timestamp[:26]
  date_time = datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f")
  #print(type(date_time))
  time_stamp = date_time.strftime("%s.%f")
  topic = "/imu/data"

  
  IMU_bag_create(data, imu_frame_id, topic, velo_timestamp)#time_stamp)   ######3 changing timestamps to velodyne 
  LiDAR_bag_create(scan, velo_frame_id, velo_topic, velo_timestamp)
  #print("time diff", velo_timestamp,time_stamp, float(velo_timestamp)- float(time_stamp))

'''
#for i, (names, subjects, marks) in enumerate(zip(names, subjects, marks)):
    #print(i, names, subjects, marks)
for count, file in enumerate (File_dir_list):
  #print(file)
  my_file = open(path+file, "r")
  # opening the file in read mode
  # reading the file
  data = my_file.read()
  data = data.split()
  imu_frame_id = "map"#'/camera_init'#'imu_link'  #'camera_init'#'imu_base'
  ##############
  timestamp = Time_stamps_list[count]
  timestamp = timestamp[:26]
  date_time = datetime.datetime.strptime(timestamp, "%Y-%m-%d %H:%M:%S.%f")
  time_stamp = date_time.strftime("%s.%f")
  topic = "/imu/data"
  IMU_bag_create(data, imu_frame_id, topic, time_stamp, count)
  times_stamps_IMU.append(time_stamp)
  ######### IMU done
#Lidar
'''

bag.close()
print("done")
