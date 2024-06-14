import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import argparse
import configparser
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import rosbag
import rospy
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import os

def bag_data_copy(srcbagpath, dstbagpath, datatopic, overwrite=False):
    if overwrite or not os.path.exists(dstbagpath):
        bagmode = 'w'
    else:
        bagmode = 'a'
    with rosbag.Bag(dstbagpath, bagmode) as dstbag:
        for topic, msg, t in rosbag.Bag(srcbagpath).read_messages(topics=[datatopic]):
            dstbag.write(datatopic, msg, t)

def bag_alldata_copy_withdt(srcbagpath, dstbagpath, dt=0):
    if not os.path.exists(dstbagpath):
        bagmode = 'w'
    else:
        bagmode = 'a'
    with rosbag.Bag(dstbagpath, bagmode) as dstbag:
        for topic, msg, t in rosbag.Bag(srcbagpath).read_messages():
            msg.header.stamp = rospy.Time.from_sec(msg.header.stamp.to_sec() + dt)
            dstbag.write(topic, msg, rospy.Time.from_sec(t.to_sec() + dt))
    

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ZMbagfile", help="input the path of ZMbag file", default='zm.bag')
    parser.add_argument("--srcbagfile", help="input the path of src bag file", default='src.bag')
    parser.add_argument("--outbagfile", help="input the path of out bag file", default='')
    parser.add_argument("--dt", help="time diff ZMbagfile and srcbagfile (ZMbagfile+dt=srcbagfile)", default=0)
    parser.add_argument("--dtfile", help="file to save time diff ZMbagfile and srcbagfile (ZMbagfile+dt=srcbagfile)", default='')
    args = parser.parse_args()
    topiclist = ['/imu/data', '/gps/fix'] ## 

    time_diff = 0
    if args.dtfile != '':
        with open(args.dtfile, 'r') as f:
            time_diff = float(f.readline().strip())
    else:
        time_diff = float(args.dt)

    outbagfile = args.outbagfile
    if os.path.exists(args.ZMbagfile) and os.path.exists(args.srcbagfile):
        if os.path.isfile(args.ZMbagfile) and os.path.isfile(args.srcbagfile):
            if outbagfile == '' or os.path.isfile(outbagfile) is False:
                outbagfile = args.ZMbagfile[:-4] + '_aligned.bag'
            if os.path.exists(outbagfile):
                os.remove(outbagfile)
            bag_alldata_copy_withdt(args.ZMbagfile, outbagfile, dt=float(time_diff))
            for topic in topiclist:
                bag_data_copy(args.srcbagfile, outbagfile, topic)
        elif os.path.isdir(args.ZMbagfile) and os.path.isdir(args.srcbagfile):
            ## find all .bag files in the ZMbagfile folder and srcbagfile folder
            ZMbagfilelist = []
            srcbagfilelist = []
            for root, dirs, files in os.walk(args.ZMbagfile):
                for file in files:
                    if file.endswith('.bag'):
                        ZMbagfilelist.append(os.path.join(root, file))
            for root, dirs, files in os.walk(args.srcbagfile):
                for file in files:
                    if file.endswith('_aligned.bag'):
                        srcbagfilelist.append(os.path.join(root, file))
            if len(ZMbagfilelist) != len(srcbagfilelist):
                print(f"ZMbagfilelist: {len(ZMbagfilelist)}, srcbagfilelist: {len(srcbagfilelist)}")
                exit()
            ## sort and copy the data from srcbagfile to ZMbagfile
            ZMbagfilelist.sort()
            srcbagfilelist.sort()
            for i in range(len(ZMbagfilelist)):
                ## 搜索basename中的数字，如果相等则copy
                ZMbasename = os.path.basename(ZMbagfilelist[i])
                srcbasename = os.path.basename(srcbagfilelist[i])
                ZMnum = int(''.join(filter(str.isdigit, ZMbasename)))
                srcnum = int(''.join(filter(str.isdigit, srcbasename)))
                print(f"ZMnum: {ZMnum}, srcnum: {srcnum}")
                if outbagfile == '' or os.path.isdir(outbagfile) is False:
                    outbagfile_tmp = ZMbagfilelist[i][:-4] + '_aligned.bag'
                else:
                    outbagfile_tmp = os.path.join(outbagfile, ZMbagfilelist[i][:-4].rsplit('/', 1)[1] + '_aligned.bag')
                if os.path.exists(outbagfile_tmp):
                    os.remove(outbagfile_tmp)
                bag_alldata_copy_withdt(ZMbagfilelist[i], outbagfile_tmp, dt=float(time_diff))
                print("outbagfile_tmp: ", outbagfile_tmp)
                # if ZMnum == srcnum:
                #     for topic in topiclist:
                #         bag_data_copy(srcbagfilelist[i], outbagfile_tmp, topic)
                # else:
                #     print(f"ZMnum: {ZMnum}, srcnum: {srcnum} not equal!")
        else:
            print(f"{args.ZMbagfile} or {args.srcbagfile} is not a file or folder!")
    else:
        print(f"{args.ZMbagfile} or {args.srcbagfile} not exists!")



