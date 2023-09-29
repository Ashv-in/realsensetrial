#!/usr/bin/python
# -*- coding: utf-8 -*-
## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2019 Intel Corporation. All Rights Reserved.
# Python 2/3 compatibility

'''
Apdated from intel's python example codes: 
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_example.py
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_rpy.py
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/t265_stereo.py
'''

'''
Saved following data from T265
(1) Captured Time
(2) Pose
(3) Orientation
(4) Velocity
(5) Angular Velocity
(6) Images
'''

# First import the library
import pyrealsense2 as rs
import cv2
import numpy as np
from math import tan, pi
import os
import argparse
import copy
from datetime import datetime
import time


# Set up a mutex to share data between threads 
from threading import Lock
frame_mutex = Lock()
frame_data = {"timestamp_ms" : None,
              "left"  : None,
              "right" : None,
              "pose" : None,
              "orientation" : None,
              "velocity" : None,
              "angular_velocity" : None,
              "acceleration": None,
              "angular_acceleration": None
              }


def callback(frame):
    global frame_data

    if frame.is_frameset():
        frameset = frame.as_frameset()
        # print(frameset. __dir__())
        f1 = frameset.get_fisheye_frame(1).as_video_frame()
        f2 = frameset.get_fisheye_frame(2).as_video_frame()
        pose = frameset.get_pose_frame()
        left_data = np.asanyarray(f1.get_data())
        right_data = np.asanyarray(f2.get_data())
        ts = frameset.get_timestamp()
        data = pose.get_pose_data()


        frame_mutex.acquire()

        frame_data["timestamp_ms"] = ts
        frame_data["left"] = left_data
        frame_data["right"] = right_data
        frame_data["pose"] = data.translation
        frame_data["orientation"] = data.rotation
        frame_data["velocity"] = data.velocity
        frame_data["angular_velocity"] = data.angular_velocity
        frame_data["acceleration"] = data.acceleration
        frame_data["angular_acceleration"] = data.angular_acceleration

        frame_mutex.release()


# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and stream everything
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)
cfg.enable_stream(rs.stream.fisheye, 1)
cfg.enable_stream(rs.stream.fisheye, 2)
cfg.enable_stream(rs.stream.accel)
cfg.enable_stream(rs.stream.gyro)

# Start streaming with our callback
pipe.start(cfg)

def main(path):
    count = 0

    # Open file for saving position data
    pose_filename = folder+'/'+'stamped_pose.txt'
    f = open(pose_filename, 'w')
    f.write('# timestamp tx ty tz qx qy qz qw \n')

    # Open file for saving velocity data
    vel_filename = folder+'/'+'stamped_vel.txt'
    vf = open(vel_filename, 'w')
    vf.write('# timestamp dx dy dz adx ady adz \n')
    
    # Open file for saving velocity data
    imu_filename = folder+'/'+'imu.txt'
    imuf = open(imu_filename, 'w')
    imuf.write('# timestamp ax ay az avx avy avz\n')

    try:
        while True:
            # Check if the camera has acquired any frames
            frames = pipe.wait_for_frames()
            callback(frames)
            frame_mutex.acquire()
            valid = frame_data["timestamp_ms"] is not None
            frame_mutex.release()

            if valid:
                # Hold the mutex only long enough to copy the stereo frames
                frame_mutex.acquire()
                frame_copy = {"timestamp_ms" : frame_data["timestamp_ms"],
                            "left"  : frame_data["left"].copy(),
                            "right" : frame_data["right"].copy(),
                            "pose" : frame_data["pose"],
                            "orientation" : frame_data["orientation"],
                            "velocity": frame_data["velocity"],
                            "angular_velocity": frame_data["angular_velocity"],
                            "acceleration" : frame_data["acceleration"],
                            "angular_acceleration" : frame_data["angular_acceleration"]
                            }
                frame_mutex.release()

                # Print output and visualize images
                if False:
                    print('============================================================================')
                    print("Captured Time[sec]: ", frame_copy["timestamp_ms"])
                    # print("Global Time in sec =",  time.time())	
                    # time.time() shows that the wall-clock time has passed approximately one second while time.clock() shows the CPU time spent on the current process is less than 1 microsecond.
                    print("Position: ", frame_copy["pose"])
                    print("Orientaton(quaternion): ", frame_copy["orientation"])
                    print("Velocity: ", frame_copy["velocity"])
                    print("Angular Velocity [radians/sec]: ", frame_copy["angular_velocity"])
                    print('============================================================================')

                    # Set up an OpenCV window to visualize the results
                    WINDOW_TITLE = 'Realsense'
                    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_NORMAL)
                    cv2.imshow(WINDOW_TITLE, frame_copy['left'])
                    key = cv2.waitKey(1)
                
                # Save the data
                # Position
                f.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n' %
                (frame_copy["timestamp_ms"]*0.001,frame_copy["pose"].x, frame_copy["pose"].y,
                 frame_copy["pose"].z, frame_copy["orientation"].x, frame_copy["orientation"].y,
                 frame_copy["orientation"].z,frame_copy["orientation"].w))

                # Velocity
                vf.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f \n' %
                (frame_copy["timestamp_ms"]*0.001,frame_copy["velocity"].x, frame_copy["velocity"].y,
                 frame_copy["velocity"].z, frame_copy["angular_velocity"].x, frame_copy["angular_velocity"].y,
                 frame_copy["angular_velocity"].z))
                
                # IMU
                imuf.write('%.12f %.12f %.12f %.12f %.12f %.12f %.12f \n' %
                (frame_copy["timestamp_ms"]*0.001,frame_copy["acceleration"].x, frame_copy["acceleration"].y,
                 frame_copy["acceleration"].z, frame_copy["angular_acceleration"].x, frame_copy["angular_acceleration"].y,
                 frame_copy["angular_acceleration"].z))

                # Fisheye Images
                dir = ['left', 'right']

                # Use this method makes saving slower (60 to 9 FPS)
                # for d in dir:
                #     path = folder+'/'+d
                #     if not os.path.isdir(path):
                #         os.makedirs(folder+'/'+d)
                #     cv2.imwrite(os.path.join(folder+'/'+d, "frame%06i.png" % count), frame_copy[d])
                
                # Use this method makes saving slower (60 to 11 FPS)
                now = time.time()
  
                for d in dir:
                    path = folder+'/'+d
                    if not os.path.isdir(path):
                        os.makedirs(folder+'/'+d)
                    # print(frame_copy[d].shape) # size: 800 * 848
                    cv2.imwrite(os.path.join(folder+'/'+d, "frame%06i.bmp" % count), frame_copy[d])
                    
#                print(time.time()-now)
#                print ("Saved data of frame # %i" % count)

                count += 1

    
    finally:
        pipe.stop()

if __name__ == '__main__':

    """
    Path of a folder to save data
    """
    parser = argparse.ArgumentParser(description="Save the data")
    parser.add_argument("--path", default='0822/test_1') #'/media/tx2/6262-3738/t265')
    args = parser.parse_args()

    # # Generate t265 folder if it dosen't exist    
    if not os.path.isdir(args.path):
        os.makedirs(args.path)

    # Generate data saving folder named by the current date and time
    # folder = args.path+'/'+datetime.now().strftime("%Y_%m_%d-%I_%M_%S_%p")
    folder = '/media/tx2/87dc40ca-1c43-48f0-85e0-0549b10881c91/t265/' + args.path
    if not os.path.isdir(folder):
        os.makedirs(folder)

    # main function
    main(folder)    
