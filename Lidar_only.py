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

def LiDAR_bag_create(scan, velo_frame_id, topic, time_stamp):
  header = Header()
  header.frame_id = velo_frame_id
  header.stamp = rospy.Time.from_sec(float(time_stamp)) #rospy.Time.from_sec(float(timestamp.strftime("%s.%f"))) #rospy.Time.from_sec(float(datetime.strftime(dt, "%s.%f")))
  # fill pcl msg
  #print("time_stamps", header.stamp)
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('i', 12, PointField.FLOAT32, 1)]
  pcl_msg = pcl2.create_cloud(header, fields, scan)
  #print("pcl_mes_timestamps", pcl_msg.header.stamp)
  #print("times_now", rospy.Time.now())
  bag.write(topic, pcl_msg, t=rospy.Time.now())#pcl_msg.header.stamp)


####
# Read files from Kitti
Main_path = '/home/parees/chao/DeepMappingPP_balance/imu_data/2011_09_30/2011_09_30_drive_0018_sync/oxts/'
path = '/home/parees/chao/DeepMappingPP_balance/imu_data/2011_09_30/2011_09_30_drive_0018_sync/oxts/data/'#os.path.join(Main_path,'data')
velo_path = '/home/parees/chao/DeepMappingPP_balance/lidar_data/dataset/sequences/10/velodyne_points'
velo_dir_path = os.path.join(velo_path,'data_bin')#'data_bin')    # use 'data_bin' for best results 
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
bag = rosbag.Bag('/home/parees/chao/bag_files/Lidar_only_full.bag', 'w', compression = rosbag.Compression.NONE)


#bar = progressbar.ProgressBar()

for i, velo_file in enumerate(velo_data_dir_list):
  if i % 50 == 0:
    print("velo", i)
  velo_filename = os.path.join(velo_dir_path, velo_file)
  scan = (np.fromfile(velo_filename, dtype=np.float32)).reshape(-1, 4)
  #print(type(scan))
  #print(scan.shape)
  #print(scan.shape[0] / 4.0)
  #pp#
  velo_topic = "/kitti/velo/pointcloud"
  velo_frame_id = "map"#'/camera_init'#'velo_link' ##'base_link'#'/camera_init'#'velo_link'
  velo_timestamp = Velo_time_stamps_list[i]
  velo_timestamp = velo_timestamp[:26]
  velo_date_time = datetime.datetime.strptime(velo_timestamp, "%Y-%m-%d %H:%M:%S.%f")
  velo_timestamp = velo_date_time.strftime("%s.%f")
  LiDAR_bag_create(scan, velo_frame_id, velo_topic, velo_timestamp)


#Lidar

bag.close()
print("done")
