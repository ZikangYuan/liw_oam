# !/usr/bin/python
#
# Convert the sensor data files in the given directory to a single rosbag.
#
# To call:
#
#   python sensordata_to_rosbag_LIW.py 2012-01-08/ 2012-01-08.bag
#

import os
# import tf
import math 
import rosbag, rospy
from tqdm import tqdm
from std_msgs.msg import Float64, UInt16, Float64MultiArray, MultiArrayDimension, MultiArrayLayout, Header
from sensor_msgs.msg import CameraInfo, Imu, PointField, NavSatStatus, NavSatFix
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import TransformStamped, TwistStamped, Transform
from cv_bridge import CvBridge

import sys
import numpy as np
import struct
# from squaternion import Quaternion
# from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R

num_hits = 1024

# q_extR = Quaternion.from_euler(0.0, 0.0, 3.1415926/2.0)
# q_extR_T =  Quaternion.from_euler(0.0, 0.0, -3.1415926/2.0)

# def write_groundtruth():
#     odom_gt = np.loadtxt(sys.argv[1] + "odometry_mu_100hz.csv", delimiter = ",")
#     print(len(odom_gt))
#     for i in range(len(odom_gt)):
#         utime = odom_gt[i, 0]


def write_gps(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS

    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status
    fix.header.stamp = timestamp
    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt

    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_fix', fix, t=timestamp)

def write_gps_rtk(gps, i, bag):

    utime = gps[i, 0]
    mode = gps[i, 1]

    lat = gps[i, 3]
    lng = gps[i, 4]
    alt = gps[i, 5]

    timestamp = rospy.Time.from_sec(utime/1e6)

    status = NavSatStatus()

    if mode==0 or mode==1:
        status.status = NavSatStatus.STATUS_NO_FIX
    else:
        status.status = NavSatStatus.STATUS_FIX

    status.service = NavSatStatus.SERVICE_GPS

    num_sats = UInt16()
    num_sats.data = gps[i, 2]

    fix = NavSatFix()
    fix.status = status
    fix.header.stamp = timestamp
    fix.latitude = np.rad2deg(lat)
    fix.longitude = np.rad2deg(lng)
    fix.altitude = alt

    track = Float64()
    track.data = gps[i, 6]

    speed = Float64()
    speed.data = gps[i, 7]

    bag.write('gps_rtk_fix', fix, t=timestamp)

def write_ms25(ms25, ms25_euler, i, bag):

    r_q = R.from_euler('zyx', [0, 0, 0], degrees=0)
    q = R.from_euler('zyx', [0, 0, 0], degrees=0).as_quat()

    r_extR = R.from_matrix([[0,-1,0],[-1,0,0],[0,0,-1]])
    q_extR = r_extR.as_quat()
    # R_imu_to_vel = ((0,-1,0),(-1,0,0),(0,0,-1))
    r_extR_T = r_extR.inv()
    q_extR_T = r_extR_T.as_quat()
    # print(r_extR_T.as_matrix())
    print(len(ms25))
    print(len(ms25_euler))
    data_lenth = len(ms25) if len(ms25) <= len(ms25_euler) else len(ms25_euler)

    i = 0

    while i < data_lenth :
        utime = ms25[i, 0]

        # mag_x = ms25[i, 1]
        # mag_y = ms25[i, 2]
        # mag_z = ms25[i, 3]
        # q = r_q.as_quat()
        # print(q_extR)
        # print(q)
        # print(q_extR_T)

        if i > 0 :
            accel_x = (ms25[i, 4] + ms25[i-1, 4]) * 0.5
            accel_y = (ms25[i, 5] + ms25[i-1, 5]) * 0.5
            accel_z = (ms25[i, 6] + ms25[i-1, 6]) * 0.5

            rot_r = (ms25[i, 7] + ms25[i-1, 7]) * 0.5
            rot_p = (ms25[i, 8] + ms25[i-1, 8]) * 0.5
            rot_h = (ms25[i, 9] + ms25[i-1, 9]) * 0.5

            r = (ms25_euler[i, 1] + ms25_euler[i, 1]) * 0.5
            p = (ms25_euler[i, 2] + ms25_euler[i, 2]) * 0.5
            h = (ms25_euler[i, 3] + ms25_euler[i, 3]) * 0.5

            r_q = R.r = R.from_euler('xyz', [h, p, r], degrees=0)
            r_lid = r_extR * r_q * r_extR_T
            q_lid = r_lid.as_quat()
            timestamp = rospy.Time.from_sec((utime + utime_last) / 2e6)

            imu = Imu()
            imu.header.frame_id = 'imu_link'
            imu.header.stamp = timestamp
            imu.linear_acceleration.x = -float(accel_y)
            imu.linear_acceleration.y = -float(accel_x)
            imu.linear_acceleration.z = -float(accel_z)
            imu.orientation.x = -q_lid[0]
            imu.orientation.y = -q_lid[1]
            imu.orientation.z = -q_lid[2]
            imu.orientation.w = -q_lid[3]
            imu.angular_velocity.x = -float(rot_p)
            imu.angular_velocity.y = -float(rot_r)
            imu.angular_velocity.z = -float(rot_h)
            bag.write('imu_raw', imu, imu.header.stamp)
        
        accel_x = ms25[i, 4]
        accel_y = ms25[i, 5]
        accel_z = ms25[i, 6]

        rot_r = ms25[i, 7]
        rot_p = ms25[i, 8]
        rot_h = ms25[i, 9]

        r = ms25_euler[i, 1]
        p = ms25_euler[i, 2]
        h = ms25_euler[i, 3]

        r_q = R.r = R.from_euler('xyz', [h, p, r], degrees=0)
        r_lid = r_extR * r_q * r_extR_T
        q_lid = r_lid.as_quat()
        
        timestamp = rospy.Time.from_sec(utime / 1e6)
        imu = Imu()
        imu.header.frame_id = 'imu_link'
        imu.header.stamp = timestamp
        imu.linear_acceleration.x = -float(accel_y)
        imu.linear_acceleration.y = -float(accel_x)
        imu.linear_acceleration.z = -float(accel_z)
        imu.orientation.x = -q_lid[0]
        imu.orientation.y = -q_lid[1]
        imu.orientation.z = -q_lid[2]
        imu.orientation.w = -q_lid[3]
        imu.angular_velocity.x = -float(rot_p)
        imu.angular_velocity.y = -float(rot_r)
        imu.angular_velocity.z = -float(rot_h)
        bag.write('imu_raw', imu, imu.header.stamp)

        utime_last = utime
        i += 1
    

def write_ms25_euler(ms25_euler, i, bag):

    utime = ms25_euler[i, 0]

    r = ms25_euler[i, 1]
    p = ms25_euler[i, 2]
    h = ms25_euler[i, 3]

    timestamp = rospy.Time.from_sec(utime/1e6)

    layout_rph = MultiArrayLayout()
    layout_rph.dim = [MultiArrayDimension()]
    layout_rph.dim[0].label = "rph"
    layout_rph.dim[0].size = 3
    layout_rph.dim[0].stride = 1

    euler = Float64MultiArray()
    euler.data = [r, p, h]
    euler.layout = layout_rph
    # bag.write('ms25_euler', euler, t=timestamp)

def write_wheels(wheels, i, bag):

    utime = wheels[i, 0]

    Lwheel = wheels[i, 1]
    Rwheel = wheels[i, 2]

    timestamp = rospy.Time.from_sec(utime/1e6)

    velo = TwistStamped()
    velo.header.stamp = timestamp
    velo.twist.linear.x = Lwheel
    velo.twist.linear.y = Rwheel

    bag.write('velocity', velo, velo.header.stamp)

def convert_vel(x_s, y_s, z_s):

    scaling = 0.005 # 5 mm
    offset = -100.0

    x = x_s * scaling + offset
    y = y_s * scaling + offset
    z = z_s * scaling + offset

    return x, -y, -z

def verify_magic(s):

    magic = 44444

    m = struct.unpack('<HHHH', s)

    return len(m)>=3 and m[0] == magic and m[1] == magic and m[2] == magic and m[3] == magic

def read_first_vel_packet(f_vel, bag):
    magic = f_vel.read(8)

    num_hits = struct.unpack('<I', f_vel.read(4))[0]
    
    utime = struct.unpack('<Q', f_vel.read(8))[0]

    f_vel.read(4) # padding

    # Read all hits
    # data = []

    for i in range(num_hits):

        x = struct.unpack('<H', f_vel.read(2))[0]
        y = struct.unpack('<H', f_vel.read(2))[0]
        z = struct.unpack('<H', f_vel.read(2))[0]
        i = struct.unpack('B', f_vel.read(1))[0]
        l = struct.unpack('B', f_vel.read(1))[0]

    return utime
    
def write_vel(f_vel,bag):
    size = os.path.getsize(sys.argv[1] + "velodyne_hits.bin")
    print(size/28/32)
    pbar = tqdm(total=size)
    num_hits = 384
    is_first = True
    last_time = 0
    last_packend_time = 0

    if is_first:
        is_first = False
        magic = f_vel.read(8)
        num_hits = struct.unpack('<I', f_vel.read(4))[0]
        last_packend_time = last_time = struct.unpack('<Q', f_vel.read(8))[0]
        f_vel.read(4) # padding
        for i in range(num_hits):
            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]
    data=[]
    while True:
        # a = f_vel.read(size-3)
        magic = f_vel.read(8)
        if len(magic) < 8:
            return

        if magic == '': # eof
            print("NO MAGIC")
            return

        if not verify_magic(magic):
            print("Could not verify magic")
            return 

        num_hits = struct.unpack('<I', f_vel.read(4))[0]
        utime = struct.unpack('<Q', f_vel.read(8))[0]
        f_vel.read(4) # padding
        pbar.update(24)
        
        # if utime > 1357847302646637:
        #     return
        
        layer_point_num = np.zeros( 32 ,dtype=np.int16)
        yaw_ind = np.zeros( (32,12) ,dtype=np.float32)
        offset_time_ind = np.zeros( (32,12) ,dtype=np.float32)
        offset_time_base = last_packend_time - last_time
        dt = float(utime - last_packend_time) / 12.0
        l_last = 0
        N = 1

        # print(utime, num_hits, offset_time_base, dt)

        for i in range(num_hits):
            x = struct.unpack('<H', f_vel.read(2))[0]
            y = struct.unpack('<H', f_vel.read(2))[0]
            z = struct.unpack('<H', f_vel.read(2))[0]
            i = struct.unpack('B', f_vel.read(1))[0]
            l = struct.unpack('B', f_vel.read(1))[0]

            if l <= l_last:
                N += 1
            
            if N>12:
                N = 12

            l_last = l
            
            # layer_point_num[l] += 1
            # offset_time_ind[l][layer_point_num[l]] = offset_time_base + dt * N
            # if layer_point_num[l] >= 12:
            #     print(l, yaw_ind[l], offset_time_ind[l])
            x, y, z = convert_vel(x, y, z)
            offset_time = int(offset_time_base + dt * N)
            if offset_time + last_time >= utime:
                offset_time = utime - last_time
            off_t = float(offset_time)
            data.append([x, y, z, offset_time, l])
            # if l == 31:
            #     print(l,offset_time_base + dt * N, int(offset_time_base + dt * N))
            # print(float(offset_time))
            # print(offset_time)
            pbar.update(8)
        
        last_packend_time = utime

        # fill pcl msg
        if utime - last_time > 1e5:
            # print(last_time / 1e6)
            # print(utime)
            header = Header()
            header.frame_id = 'velodyne'
            header.stamp = rospy.Time.from_sec(last_time/1e6)
            fields = [PointField('x', 0, PointField.FLOAT32, 1),
                    PointField('y', 4, PointField.FLOAT32, 1),
                    PointField('z', 8, PointField.FLOAT32, 1),
                    # PointField('intensity', 12, PointField.FLOAT32, 1),
                    PointField('time', 16, PointField.FLOAT32, 1),
                    PointField('ring', 20, PointField.UINT16, 1)]
            pcl_msg = pcl2.create_cloud(header, fields, data)
            pcl_msg.is_dense = True
            bag.write("points_raw", pcl_msg, t=pcl_msg.header.stamp)
            last_time = utime
            data=[]

def main(args):

    if len(sys.argv) < 2:
        print('Please specify sensor data directory file')
        return 1

    if len(sys.argv) < 3:
        print('Please specify output rosbag file')
        return 1

    bag = rosbag.Bag(sys.argv[2], 'w')

    gps = np.loadtxt(sys.argv[1] + "gps.csv", delimiter = ",")
    gps_rtk = np.loadtxt(sys.argv[1] + "gps_rtk.csv", delimiter = ",")
    ms25 = np.loadtxt(sys.argv[1] + "ms25.csv", delimiter = ",")
    ms25_euler = np.loadtxt(sys.argv[1] + "ms25_euler.csv", delimiter = ",")
    wheels = np.loadtxt(sys.argv[1] + "wheels.csv", delimiter = ",")

    i_gps = 0
    i_gps_rtk = 0
    i_ms25 = 0
    i_ms25_euler = 0
    i_wheels = 0

    f_vel = open(sys.argv[1] + "velodyne_hits.bin", "rb")

    # time_last = read_first_vel_packet(f_vel, bag)
    # data = []
    # utime_vel = time_last

    # write_groundtruth()
    write_vel(f_vel, bag)
    write_ms25(ms25, ms25_euler, i_ms25, bag)

    print('Loaded data, writing ROSbag...')

    count = 0

    while 1:
        # Figure out next packet in time
        next_packet = "done"
        next_utime = -1 # 1357847302646637 
        count = count + 1

        # print(next_utime - utime_vel)


        if i_gps<len(gps) and (gps[i_gps, 0]<next_utime or next_utime<0):
            next_packet = "gps"

        if i_gps_rtk<len(gps_rtk) and (gps_rtk[i_gps_rtk, 0]<next_utime or next_utime<0):
            next_packet = "gps_rtk"

        if i_ms25<len(ms25) and (ms25[i_ms25, 0]<next_utime or next_utime<0):
            next_packet = "ms25"

        if i_ms25_euler<len(ms25_euler) and (ms25_euler[i_ms25_euler, 0]<next_utime or next_utime<0):
            next_packet = "ms25_euler"

        if i_wheels<len(wheels) and (wheels[i_wheels, 0]<next_utime or next_utime<0):
            next_packet = "wheels"

        # if utime_vel>0 and (utime_vel<next_utime or next_utime<0):
        #     next_packet = "vel"

        # if utime_hok30>0 and (utime_hok30<next_utime or next_utime<0):
        #     next_packet = "hok30"

        # if utime_hok4>0 and (utime_hok4<next_utime or next_utime<0):
        #     next_packet = "hok4"

        # Now deal with the next packet
        if next_packet == "done":
            break
        elif next_packet == "gps":
            print("Percentage: {0}% \r".format(i_gps * 100.0 / len(gps[:,1])))
            write_gps(gps, i_gps, bag)
            i_gps = i_gps + 1
        elif next_packet == "gps_rtk":
            write_gps_rtk(gps_rtk, i_gps_rtk, bag)
            i_gps_rtk = i_gps_rtk + 1
        elif next_packet == "ms25":
            # write_ms25(ms25, ms25_euler, i_ms25, bag)
            i_ms25 = i_ms25 + 1
        elif next_packet == "ms25_euler":
            # write_ms25_euler(ms25_euler, i_ms25_euler, bag)
            i_ms25_euler = i_ms25_euler + 1
        elif next_packet == "wheels":
            write_wheels(wheels, i_wheels, bag)
            i_wheels = i_wheels + 1
        # elif next_packet == "vel":
            # time_last, utime_vel, data = read_next_vel_packet(f_vel,time_last,data,bag)
        else:
            print("Unknown packet type")

    f_vel.close()
    # f_hok_30.close()
    # f_hok_4.close()
    bag.close()

    return 0

if __name__ == '__main__':
    sys.exit(main(sys.argv))
