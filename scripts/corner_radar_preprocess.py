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

def update_plot(frame_num, data, ax):
    if frame_num >= len(data):
        return
    if frame_num%2==0:
        ax.clear()
    data_tmp = np.array(data[frame_num])
    lim_xy = 100
    ax.plot(data_tmp[:, 0], data_tmp[:, 1], 'b.')
    ax.set_title(f"Frame {frame_num + 1}") 
    ax.set_xlim([-lim_xy, lim_xy])  # 设置 x 轴范围
    ax.set_ylim([-lim_xy, lim_xy])  # 设置 y 轴范围
    ax.set_aspect('equal')  # 设置比例，使得 x 和 y 的单位长度相同

def update_plot_all_sensors(frame_num, data, axes):
    for i in range(4):
        if frame_num >= len(data[i]):
            print("No data for radar_{}".format(i+1))
            continue
        axes[i].clear()
        data_tmp = np.array(data[i][frame_num])
        axes[i].plot(data_tmp[:, 0], data_tmp[:, 1], 'b.')
        axes[i].set_title(f"Frame {frame_num + 1}")
        axes[i].set_xlim([0, 100])
        axes[i].set_ylim([-60, 60])
        axes[i].set_aspect('equal')

def read_csv_data(datafile, visualize=False):
    # load data
    data = np.loadtxt(datafile, delimiter=',', skiprows=1)
        
    sensors_frames = [[], [], [], []] # four sensors
    sensors_frames_ts = [[], [], [], []]
    sensors_frames_count = [0, 0, 0, 0]
    cur_sensor_frame = []
    last_time = -1
    last_sensor_id = -1
    for i in range(data.shape[0]):
        sensor_id = data[i, 4]
        # radar_time = data[i, 3]/1000  # same as DGPS time
        radar_time = data[i, -8]/1000
        computer_time = data[i, -3]/1000
        if sensor_id != last_sensor_id and last_sensor_id != -1:
            # save
            indices = [index for index, value in enumerate(sensors_frames_ts[int(last_sensor_id) - 1]) if value == last_time]
            if indices:
                sensors_frames[int(last_sensor_id) - 1][indices[0]] += cur_sensor_frame
            else:
                sensors_frames[int(last_sensor_id) - 1].append(cur_sensor_frame)
                sensors_frames_ts[int(last_sensor_id) - 1].append(last_time)
                sensors_frames_count[int(last_sensor_id) - 1] += 1
            cur_sensor_frame = []
            
        ## data analyze
        snr_db = data[i, 6]
        r, v, ad, ed = data[i, 7:11]
        a = ad * np.pi / 180.0
        e = ed * np.pi / 180.0
        x = r * np.cos(e) * np.cos(a)
        y = r * np.cos(e) * np.sin(a)
        z = r * np.sin(e)
        cur_sensor_frame.append([x, y, z, v, snr_db, r, a, e, radar_time, computer_time, sensor_id])
        last_sensor_id = sensor_id
        # last_time = radar_time
        last_time = computer_time

    indices = [index for index, value in enumerate(sensors_frames_ts[int(last_sensor_id) - 1]) if value == last_time]
    if indices:
        sensors_frames[int(last_sensor_id) - 1][indices[0]] += cur_sensor_frame
    else:
        sensors_frames[int(last_sensor_id) - 1].append(cur_sensor_frame)
        sensors_frames_ts[int(last_sensor_id) - 1].append(last_time)
        sensors_frames_count[int(last_sensor_id) - 1] += 1

    print("all sensors is points number is: ", sensors_frames_count)

    if visualize:
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(1, 4, figsize=(18, 6), sharex=True)
        axes = [ax1, ax2, ax3, ax4]
        ani = FuncAnimation(fig, update_plot_all_sensors, frames=100000, fargs=(sensors_frames, axes), repeat=False)
        plt.tight_layout()
        plt.show()

    return sensors_frames, sensors_frames_ts

def read_ini_params(inifile):
    config = configparser.ConfigParser(allow_no_value=True)

    # 从文件中读取配置信息
    with open(inifile, 'r') as file:
        config.read_file(file)

    # 遍历读取 Radar 配置
    radars_ini_params = []
    for section in config.sections():
        print(f"[{section}]")
        ini_params_tmp = []
        for key, value in config.items(section):
            ini_params_tmp.append(float(value))
            print(f"{key} = {value}")
        radars_ini_params.append(ini_params_tmp)
        print()  # 输出空行，用于分隔每个 Radar 的配置

    ## save radars_ini_params
    with open("radars_ini_params.txt", 'w') as f:
        for i in range(len(radars_ini_params)):
            if i==0:
                continue
            roll = radars_ini_params[i][3]
            pitch = radars_ini_params[i][2]
            yaw = -radars_ini_params[i][1]  ## 左负==>右负
            tx = radars_ini_params[i][4]
            ty = radars_ini_params[i][5]
            tz = radars_ini_params[i][6]
            f.write(f"{tx} {ty} {tz} {roll} {pitch} {yaw}\n")
    
    return radars_ini_params

def regis_all_sensors_data(radars_ini_params, sensors_frames, sensors_frames_ts, visualize=False):
    uniformed_coor_frames = []
    uniformed_coor_frames_ts = []
    # for i in range(len(radars_ini_params)):
    for i in range(4):
        # print(len(radars_ini_params))
        if len(sensors_frames[i]) == 0:
            continue
        if i>5:
            continue
        roll = radars_ini_params[i][3]
        pitch = radars_ini_params[i][2]
        yaw = -radars_ini_params[i][1]  ## 左负==>右负
        tx = radars_ini_params[i][4]
        ty = radars_ini_params[i][5]
        tz = radars_ini_params[i][6]
        # convert roll, pitch, yaw to rotation matrix
        R = Rotation.from_euler('zyx', [yaw, pitch, roll], degrees=False)
        rotation_matrix = R.as_matrix()
        translation_vector = np.array([tx, ty, tz])
        for j in range(len(sensors_frames[i])):
            sensors_frames[i][j] = np.array(sensors_frames[i][j])
            # print(sensors_frames[i][j].shape)
            sensors_frames[i][j] = sensors_frames[i][j].T
            sensors_frames[i][j][:3, :] = rotation_matrix @ sensors_frames[i][j][:3, :] + translation_vector.reshape(3, 1)
            indices = [index for index, value in enumerate(uniformed_coor_frames_ts) if value == sensors_frames_ts[i][j]]
            if indices:
                uniformed_coor_frames[indices[0]] += sensors_frames[i][j].T.tolist()
            else:
                uniformed_coor_frames.append(sensors_frames[i][j].T.tolist())
                uniformed_coor_frames_ts.append(sensors_frames_ts[i][j])

    ## sort uniformed_coor_frames by uniformed_coor_frames_ts
    uniformed_coor_frames = [x for _, x in sorted(zip(uniformed_coor_frames_ts, uniformed_coor_frames))]
    uniformed_coor_frames_ts = sorted(uniformed_coor_frames_ts)

    ## cluster the uniformed_coor_frames by uniformed_coor_frames_ts while diff<0.1s
    uniformed_coor_frames_tmp = []
    uniformed_coor_frames_ts_tmp = []
    uniformed_coor_frames_tmp.append(uniformed_coor_frames[0])
    uniformed_coor_frames_ts_tmp.append(uniformed_coor_frames_ts[0])
    for i in range(1, len(uniformed_coor_frames)):
        if uniformed_coor_frames_ts[i] - uniformed_coor_frames_ts_tmp[-1] < 0.1:
            uniformed_coor_frames_tmp[-1] += uniformed_coor_frames[i]
        else:
            uniformed_coor_frames_tmp.append(uniformed_coor_frames[i])
            uniformed_coor_frames_ts_tmp.append(uniformed_coor_frames_ts[i])
    uniformed_coor_frames = uniformed_coor_frames_tmp
    uniformed_coor_frames_ts = uniformed_coor_frames_ts_tmp

    # ## plot x and y of uniformed_coor_frames
    # for i in range(len(uniformed_coor_frames)):
    #     data_temp = np.array(uniformed_coor_frames[i])
    #     # 判断 mean(abs(data_temp[:, 3]))<1:
    #     if np.mean(np.abs(data_temp[:, 3])) < 1:
    #         continue
    #     plt.plot(data_temp[:, 0], data_temp[:, 3], 'b.')
    #     plt.title(f"Frame {i + 1}")
    #     plt.show()

    if visualize:
        fig, ax = plt.subplots()
        ani = FuncAnimation(fig, update_plot, frames=100000, fargs=(uniformed_coor_frames, ax), repeat=False)
        plt.tight_layout()
        plt.show()

    return uniformed_coor_frames, uniformed_coor_frames_ts

fields = [
    PointField('x', 0, PointField.FLOAT32, 1),
    PointField('y', 4, PointField.FLOAT32, 1),
    PointField('z', 8, PointField.FLOAT32, 1),
    PointField('doppler', 16, PointField.FLOAT32, 1),
    PointField('intensity', 20, PointField.FLOAT32, 1),
    PointField('r', 24, PointField.FLOAT32, 1),
    PointField('a', 28, PointField.FLOAT32, 1),
    PointField('e', 32, PointField.FLOAT32, 1),
    PointField('sensor_time', 36, PointField.FLOAT32, 1),
    PointField('computer_time', 40, PointField.FLOAT32, 1),
    PointField('sensor_id', 44, PointField.FLOAT32, 1)
]
def convert_data_to_bag(outbagpath, uniformed_coor_frames, uniformed_coor_frames_ts, pctopic = "/corner_radar", overwrite=False):
    if overwrite or not os.path.exists(outbagpath):
        bagmode = 'w'
    else:
        bagmode = 'a'
    with rosbag.Bag(outbagpath, bagmode) as bag:
        for i in range(len(uniformed_coor_frames)):
            header = Header()
            header.stamp = rospy.Time(uniformed_coor_frames_ts[i])
            header.frame_id = "radar"
            msg = point_cloud2.create_cloud(header, fields, uniformed_coor_frames[i])
            msg.is_dense = True
            bag.write(pctopic, msg, header.stamp)

def recursive_preprocess(datadir, inifile, outbagdir=""):
    ## read ini file
    radars_ini_params = read_ini_params(inifile)
    del radars_ini_params[0]  # radar0 not exist
    ## read data file
    for root, dirs, files in os.walk(datadir):
        datafragmentsfile = []
        for file in files:
            if file.endswith("_PointCloud.csv"):
                datafragmentsfile.append(os.path.join(root, file))
        if len(datafragmentsfile)==0:
            print("No _PointCloud.csv file found in ", root, "! Skip")
            continue
        if outbagdir=="":
            if root.endswith("/"):
                root = root[:-1]
            savedir = root.rsplit('/', 1)[0]
            outbagpath = os.path.join(savedir, os.path.basename(root)+".bag")
            print("outbagpath: ", outbagpath)
        else:
            if root.endswith("/"):
                root = root[:-1]
            outbagpath = os.path.join(outbagdir, os.path.basename(root)+".bag")
        print(root, "is processing...")
        uniformed_coor_frames_all = []
        uniformed_coor_frames_ts_all = []
        for fragmentfile in datafragmentsfile:
            print("cur_processing dir is:", fragmentfile)
            sensors_frames, sensors_frames_ts = read_csv_data(fragmentfile, visualize=False)
            uniformed_coor_frames, uniformed_coor_frames_ts = regis_all_sensors_data(radars_ini_params, sensors_frames, sensors_frames_ts, visualize=False)
            uniformed_coor_frames_all += uniformed_coor_frames
            uniformed_coor_frames_ts_all += uniformed_coor_frames_ts
        ## convert to bag file
        print("convert to bagpath ", outbagpath, "...")
        convert_data_to_bag(outbagpath, uniformed_coor_frames_all, uniformed_coor_frames_ts_all, pctopic = "/corner_radar", overwrite=True)

def bag_data_copy(srcbagpath, dstbagpath, datatopic, overwrite=False):
    if overwrite or not os.path.exists(dstbagpath):
        bagmode = 'w'
    else:
        bagmode = 'a'
    with rosbag.Bag(dstbagpath, bagmode) as dstbag:
        for topic, msg, t in rosbag.Bag(srcbagpath).read_messages(topics=[datatopic]):
            dstbag.write(datatopic, msg, t)
    

if __name__=="__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--datafile", help="input the path of the data file", default='data.txt')
    parser.add_argument("--inifile", help="input the path of the ini params file", default='ini.ini')
    parser.add_argument("--outbagpath", help="input the path of the output bag file", default='corner_radar.bag')
    args = parser.parse_args()

    if os.path.exists(args.inifile)==False:
        print(f"{args.inifile} can not be found!")
        exit(0)

    if os.path.exists(args.datafile):
        # radars_ini_params = read_ini_params(args.inifile)
        # radars_ini_params = np.array(radars_ini_params)
        # print("radars_ini_params: \n", radars_ini_params)
        # fig, ax = plt.subplots()
        # ax.plot(radars_ini_params[:,4], radars_ini_params[:,5], 'b.')
        # ax.set_title("radars_ini_params")
        # ax.set_aspect('equal')
        # plt.show()
        if os.path.isfile(args.datafile):
            radars_ini_params = read_ini_params(args.inifile)
            del radars_ini_params[0]  # radar0 not exist
            sensors_frames, sensors_frames_ts = read_csv_data(args.datafile, visualize=False)
            uniformed_coor_frames, uniformed_coor_frames_ts = regis_all_sensors_data(radars_ini_params, sensors_frames, sensors_frames_ts, visualize=False)
            convert_data_to_bag(args.outbagpath, uniformed_coor_frames, uniformed_coor_frames_ts, pctopic = "/corner_radar", overwrite=True)
        elif os.path.isdir(args.datafile):
            recursive_preprocess(args.datafile, args.inifile)
    else:
        print(f"{args.datafile} can not be found!")



