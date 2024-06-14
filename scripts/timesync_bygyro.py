import matplotlib.pyplot as plt
import numpy as np
import argparse
from scipy.spatial.transform import Rotation
import rosbag
import os
from scipy.signal import butter, filtfilt, correlate
from scipy.fft import fft, fftfreq

def get_gyro_frombag(srcbagpath, imu_topic='/imu/data'):
    ts = []
    gyrodata = []
    for topic, msg, t in rosbag.Bag(srcbagpath).read_messages(topics=[imu_topic]):
        ts.append(msg.header.stamp.to_sec())
        gyrodata.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    return np.array(ts), np.array(gyrodata)

def computer_gyro_from_tumpose(tumposefile):
    ## pose: t, x, y, z, qx, qy, qz, qw
    t = []
    quat = []
    gyrodata = []
    with open(tumposefile, 'r') as f:
        for line in f.readlines():
            line = line.strip()
            if line.startswith('#'):
                continue
            else:
                line = line.split()
                t.append(float(line[0]))
                quat.append([float(line[4]), float(line[5]), float(line[6]), float(line[7])])
    quat = np.array(quat)
    for i in range(quat.shape[0]-1):
        R_cur = Rotation.from_quat(quat[i, :]).as_matrix()
        R_next = Rotation.from_quat(quat[i+1, :]).as_matrix()
        dt = t[i+1] - t[i]
        dR = R_cur.T @ R_next
        gyro_tmp = Rotation.from_matrix(dR).as_rotvec() / dt
        gyrodata.append(gyro_tmp)
    return np.array(t[:-1]), np.array(gyrodata)

def highpass_filter(data, cutoff_freq, sample_rate, order=4):
    # 设计巴特沃斯高通滤波器
    b, a = butter(order, cutoff_freq / (0.5 * sample_rate), btype='high', analog=False)
    # 应用滤波器
    filtered_data = filtfilt(b, a, data, axis=0)
    return filtered_data

def lowpass_filter(data, cutoff_freq, sample_rate, order=4):
    # 设计巴特沃斯低通滤波器
    b, a = butter(order, cutoff_freq / (0.5 * sample_rate), btype='low', analog=False)
    # 应用滤波器
    filtered_data = filtfilt(b, a, data, axis=0)
    return filtered_data 

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--tumposefile", help="input the path of tumpose file", default='tum.txt')
    parser.add_argument("--srcbagfile", help="input the path of src bag file", default='data.bag')
    args = parser.parse_args()

    if os.path.exists(args.tumposefile) and os.path.exists(args.srcbagfile) and os.path.isfile(args.tumposefile) and os.path.isfile(args.srcbagfile):
        t, gyrodata = computer_gyro_from_tumpose(args.tumposefile)
        gyrodata = lowpass_filter(gyrodata, 10, 100)
        gyrodata_norm = np.linalg.norm(gyrodata, axis=1)
        t_bag, gyrodata_bag = get_gyro_frombag(args.srcbagfile)
        gyrodata_bag = lowpass_filter(gyrodata_bag, 10, 100)
        gyrodata_bag_norm = np.linalg.norm(gyrodata_bag, axis=1)
        print(f"t: {t.shape}, gyrodata: {gyrodata.shape}")
        print(f"t_bag: {t_bag.shape}, gyrodata_bag: {gyrodata_bag.shape}")

        ## 计算t和t_bag的全集t_overlap，并采样为len(t)个, 采样t_overlap以及对应时刻的gyrodata_norm, gyrodata_bag_norm
        min_time = min(t[0], t_bag[0])
        max_time = max(t[-1], t_bag[-1])
        t_overlap = np.linspace(min_time, max_time, len(t))
        gyrodata_norm_overlap = np.interp(t_overlap, t, gyrodata_norm)
        gyrodata_bag_norm_overlap = np.interp(t_overlap, t_bag, gyrodata_bag_norm)
        ## 计算信号的互相关,并得到dt
        corr = correlate(gyrodata_norm_overlap, gyrodata_bag_norm_overlap, mode='full')
        peak_index = np.argmax(corr)
        time_shift = np.linspace(min_time - max_time, max_time - min_time, len(corr))
        estimated_time_offset = time_shift[peak_index]
        dt = -estimated_time_offset
        print(f"dt: {dt}")

        # save dt at tumpose file folder
        tumposefilefolder = os.path.dirname(args.tumposefile)
        dtfile = os.path.join(tumposefilefolder, 'dt.txt')
        with open(dtfile, 'w') as f:
            f.write(str(dt))

        plt.figure()
        plt.plot(t, gyrodata_norm, 'b', label='norm_cornerradar')
        plt.plot(t+dt, gyrodata_norm, 'r', label='norm_cornerradar_corr')
        plt.plot(t_bag, gyrodata_bag_norm, 'g', label='norm_bagimu')
        plt.legend()
        plt.show()
    else:
        print("file not exist or not a file!")
        exit()



