import argparse
import os
import rosbag
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import copy

def extract_pcd_withimu(bagfile, pcd_topic, imu_topic, pcddir, start_time=-1):
    """
    extract a pointcloud2 message from rosbag while have imu at the same time, and save as a pcd
    """
    bag = rosbag.Bag(bagfile)

    # Inserted code starts here
    pointcloud2_msg = None
    imu_msg = None
    first_scan_t = 0
    for topic, msg, t in bag.read_messages(topics=[pcd_topic, imu_topic]):
        if start_time != -1 and t.to_sec() < start_time:
            continue
        if topic == pcd_topic and imu_msg is not None:
            pointcloud2_msg = msg
        elif topic == imu_topic:
            imu_msg = msg
        if pointcloud2_msg is not None and imu_msg is not None:
            first_scan_t =t
            break

    if pointcloud2_msg is not None and imu_msg is not None:
        # Save pointcloud2 message as pcd
        gen = pc2.read_points(pointcloud2_msg, skip_nans=True)
        int_data = list(gen)
        xyz = np.array([[0, 0, 0]])
        for x in int_data:
            xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
        out_pcd = o3d.geometry.PointCloud()
        out_pcd.points = o3d.utility.Vector3dVector(xyz)
        output_filename = os.path.join(pcddir, str(first_scan_t) + ".pcd")
        o3d.io.write_point_cloud(output_filename, out_pcd)
    else:
        print("Error: Pointcloud2 message or IMU message not found in the bag file.")

def merge_pose_as_traj(posefolder):
    ## read all poses.txt in the pose folder
    posefilelist = []
    for root, dirs, files in os.walk(posefolder):
        for file in files:
            if file.endswith('.txt'):
                posefilelist.append(os.path.join(root, file))
    ## merge all poses.txt into a traj.txt
    traj = []
    pose_count = 0
    for posefile in posefilelist:
        with open(posefile, 'r') as f:
            for line in f.readlines():
                ## 空格分割，取3，7，11 为x,y,z， 剩余部分为旋转矩阵[0,1,2;4,5,6;8,9,10]
                pose = line.strip().split(' ')
                pose = [float(x) for x in pose]
                x = pose[3]
                y = pose[7]
                z = pose[11]
                Rot_mat = np.array(pose[0:3] + pose[4:7] + pose[8:11]).reshape(3,3)
                # covert to quaternion
                r = Rotation.from_matrix(Rot_mat)
                quat = r.as_quat()
                traj.append([pose_count, x, y, z, quat[0], quat[1], quat[2], quat[3]])
                pose_count += 1
    
    ## save traj.txt
    traj = np.array(traj)
    root_dir = os.path.dirname(os.path.dirname(posefolder))
    np.savetxt(os.path.join(root_dir, 'traj_tum.txt'), traj, fmt='%d %f %f %f %f %f %f %f')

def read_tum(pose_file, visualize=False, split_format=' '):
    # 读取位姿数据
    with open(pose_file, 'r') as file:
        lines = file.readlines()

    timestamps = []  # 存储时间戳
    poses = []       # 存储位姿数据
    for line in lines:
        data = line.strip().split(split_format)
        if(data[0].startswith('#')):
            continue
        if len(data) >= 8:  # 确保每行有8个数据项
            timestamp = float(data[0])
            tx, ty, tz, qx, qy, qz, qw = map(float, data[1:8])
            timestamps.append(timestamp)
            poses.append([tx, ty, tz, qx, qy, qz, qw])

    # 将位姿数据转换为NumPy数组
    poses = np.array(poses)

    # 提取xyz坐标
    x = poses[:, 0]
    y = poses[:, 1]
    z = poses[:, 2]

    # 提取旋转信息（四元数）
    qx = poses[:, 3]
    qy = poses[:, 4]
    qz = poses[:, 5]
    qw = poses[:, 6]

    timestamps = np.array(timestamps)
    # timestamps = timestamps-timestamps[0]
    
    if visualize:
        # 创建轨迹图
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

        # 绘制xyz坐标轨迹
        ax1.plot(timestamps, x, label='X', linewidth=2)
        ax1.plot(timestamps, y, label='Y', linewidth=2)
        ax1.plot(timestamps, z, label='Z', linewidth=2)
        ax1.set_ylabel('Position (XYZ)')
        ax1.set_title('Position Trajectory')
        ax1.legend()
        ax1.grid(True)

        # 绘制旋转信息轨迹
        ax2.plot(timestamps, qx, label='qx', linewidth=2)
        ax2.plot(timestamps, qy, label='qy', linewidth=2)
        ax2.plot(timestamps, qz, label='qz', linewidth=2)
        ax2.plot(timestamps, qw, label='qw', linewidth=2)
        ax2.set_xlabel('Time')
        ax2.set_ylabel('Quaternion Components')
        ax2.set_title('Quaternion Trajectory')
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

    return timestamps, poses

def covert_tumpose_to_matrix(pose):
    x = pose[0]
    y = pose[1]
    z = pose[2]
    qx = pose[3]
    qy = pose[4]
    qz = pose[5]
    qw = pose[6]
    Rot_mat = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
    return np.array([[Rot_mat[0,0], Rot_mat[0,1], Rot_mat[0,2], x],
                     [Rot_mat[1,0], Rot_mat[1,1], Rot_mat[1,2], y],
                     [Rot_mat[2,0], Rot_mat[2,1], Rot_mat[2,2], z],
                     [0, 0, 0, 1]])

def convert_matrix_to_tumpose(mat):
    Rot_mat = mat[0:3, 0:3]
    r = Rotation.from_matrix(Rot_mat)
    quat = r.as_quat()
    return [mat[0,3], mat[1,3], mat[2,3], quat[0], quat[1], quat[2], quat[3]]

def place_reloc_by_traj(trajfile, dbcentersfile, init_trans_mat, match_dist=30, Visualize=False, split_format1=' ', split_format2=' '):
    ## read trajfile, tum format
    timestamps, poses = read_tum(trajfile, split_format=split_format1)
    ## read dbcentersfile
    dbtimestamps, dbcenters = read_tum(dbcentersfile, split_format=split_format2)
    ## transform traj to dbcenters by init_tranform
    traj = []
    for pose in poses:
        pose_convert = convert_matrix_to_tumpose(init_trans_mat @ covert_tumpose_to_matrix(pose))
        traj.append(pose_convert)
    traj = np.array(traj)
    ## find the nearest pose of traj in dbcenters
    cover_timestamps = []
    traj_match_idx = []
    match_traj_pose = []
    match_db_pose = []
    count = 0
    for pose in traj:
        pose = np.array(pose)
        pose = pose.reshape(1,7)
        pose = np.tile(pose, (len(dbcenters), 1))
        dist_square = np.sum((dbcenters[:,0:2] - pose[:,0:2])**2, axis=1)
        idx = np.argmin(dist_square)
        min_dist = np.sqrt(dist_square[idx])
        if min_dist < match_dist:
            cover_timestamps.append(timestamps[count])
            traj_match_idx.append([timestamps[count], dbtimestamps[idx], min_dist])
            match_traj_pose.append(traj[count])
            match_db_pose.append(dbcenters[idx])
        count += 1
    traj_match_idx = np.array(traj_match_idx)
    match_traj_pose = np.array(match_traj_pose)
    match_db_pose = np.array(match_db_pose)
    ## plot the pos of traj, dbcenters, traj_match
    if Visualize:
        plt.figure()
        plt.plot(traj[:,0], traj[:,1], label='traj')
        plt.plot(dbcenters[:,0], dbcenters[:,1], label='dbcenters')
        # draw line between match traj and dbcenters
        for i in range(len(traj_match_idx)):
            plt.plot([match_traj_pose[i,0], match_db_pose[i,0]], [match_traj_pose[i,1], match_db_pose[i,1]], 'r')
        plt.legend()
        plt.show()

    ## sort and cluster cover_timestamps and get begin time and end time of each cluster
    cover_timestamps = np.array(cover_timestamps)
    cover_timestamps.sort()
    cluster_timestamps_idx = []
    cluster_timestamps_idx_tmp = []
    cluster_timestamps_idx_tmp.append(0)
    for i in range(1, len(cover_timestamps)):
        if cover_timestamps[i] - cover_timestamps[i-1] > 0.5:
            cluster_timestamps_idx.append(cluster_timestamps_idx_tmp)
            cluster_timestamps_idx_tmp = []
            cluster_timestamps_idx_tmp.append(i)
        else:
            cluster_timestamps_idx_tmp.append(i)
    cluster_timestamps_idx.append(cluster_timestamps_idx_tmp)
    ## get begin time, end time of each cluster, and velocity of begin time
    cluster_timestamps_vel = []
    for i in range(len(cluster_timestamps_idx)):
        begin_idx = cluster_timestamps_idx[i][0]
        end_idx = cluster_timestamps_idx[i][-1]
        begin_vel = np.sqrt((match_traj_pose[begin_idx,0] - match_traj_pose[begin_idx+1,0])**2 + (match_traj_pose[begin_idx,1] - match_traj_pose[begin_idx+1,1])**2) / (cover_timestamps[begin_idx+1] - cover_timestamps[begin_idx]) 
        print(f"cluster {i}: {cover_timestamps[begin_idx]-timestamps[0]}, {cover_timestamps[end_idx]-cover_timestamps[begin_idx]}, {begin_vel}")
        cluster_timestamps_vel.append([cover_timestamps[begin_idx]-timestamps[0], cover_timestamps[end_idx]-cover_timestamps[begin_idx], begin_vel])
    return cluster_timestamps_vel, timestamps[0]
    

def merge_project1_2(project1_file, project2_file, output_file):
    ## copy project1_file/regis and project1_file/trans to output_file
    os.system(f"cp -r {project1_file}/regis {output_file}")
    os.system(f"cp -r {project1_file}/trans {output_file}")
    num_project1 = 61
    ## read i_uniform.pcd in project2_file/regis and copy&rename them to output_file/regis as num_project1+i.pcd.
    regis2dir = os.path.join(project2_file, 'regis')
    for root, dirs, files in os.walk(regis2dir):
        for file in files:
            if file.endswith('_uniform.pcd'):
                seq_num = int(''.join(filter(str.isdigit, file)))
                # print(f"seq_num: {file}, {seq_num}")
                srcfile = os.path.join(root, file)
                dstfile = os.path.join(output_file, 'regis', str(num_project1+seq_num) + '.pcd')
                os.system(f"cp {srcfile} {dstfile}")
    
    trans2dir = os.path.join(project2_file, 'trans/uniform_trans/')
    for root, dirs, files in os.walk(trans2dir):
        for file in files:
            if file.endswith('.txt'):
                seq_num = int(''.join(filter(str.isdigit, file)))
                # print(f"seq_num: {file}, {seq_num}")
                srcfile = os.path.join(root, file)
                dstfile = os.path.join(output_file, 'trans/absolute_pgo/', str(num_project1+seq_num) + '.txt')
                os.system(f"cp {srcfile} {dstfile}")

    ## 拼接centers.txt and poses.txt
    centers1 = np.loadtxt(os.path.join(project1_file, 'regis/centers.txt'))
    centers2 = np.loadtxt(os.path.join(project2_file, 'regis/centers.txt'))
    for i in range(len(centers2)):
        centers2[i,0] += num_project1
    centers = np.vstack((centers1, centers2))
    np.savetxt(os.path.join(output_file, 'regis/centers.txt'), centers, fmt='%d %f %f %f %f %f %f %f')
    # poses1 = np.loadtxt(os.path.join(project1_file, 'regis/pose.txt'))
    # poses2 = np.loadtxt(os.path.join(project2_file, 'regis/pose.txt'))
    # for i in range(len(poses2)):
    #     poses2[i,0] += num_project1
    # poses = np.vstack((poses1, poses2))
    # np.savetxt(os.path.join(output_file, 'regis/pose.txt'), poses, fmt='%d %f %f %f %f %f %f %f')

def rigid_transform_3D(A, B):
    """
    Calculate the rigid transformation matrix between point sets A and B.
    Assumes A and B are both 3D points (rows are points, columns are xyz).
    """
    assert len(A) == len(B)

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    H = np.dot((A - centroid_A).T, B - centroid_B)

    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = np.dot(Vt.T, U.T)

    t = -np.dot(R, centroid_A) + centroid_B

    T = np.identity(4)
    T[:3, :3] = R
    T[:3, 3] = t

    return T

def objective_function(x, A, B):
    """
    Objective function for the optimization.
    x: flattened 4x4 transformation matrix
    A, B: Point sets
    """
    T = x.reshape((4, 4))
    transformed_A = np.dot(A, T[:3, :3].T) + T[:3, 3]
    return np.sum(np.square(transformed_A - B))

def find_transform_matrix(A, B):
    """
    Find the rigid transformation matrix between point sets A and B.
    """
    x0 = np.identity(4).flatten()  # Initial guess: identity matrix

    result = minimize(objective_function, x0, args=(A, B), method='BFGS')
    
    if result.success:
        return result.x.reshape((4, 4))
    else:
        raise ValueError("Transformation optimization failed.")
    
def find_nearest_timestamp(timestamp, timestamps):
    idx = np.argmin(np.abs(timestamps - timestamp))
    return timestamps[idx]

def traj_align(trajfile1, trajfile2, split_format1=' ', split_format2=' '):
    ## read trajfile1, tum format
    timestamps1, poses1 = read_tum(trajfile1, split_format=split_format1)
    ## read trajfile2, tum format
    timestamps2, poses2 = read_tum(trajfile2, split_format=split_format2)
    ## find the nearest timestamp of trajfile2 in trajfile1 and match them
    aligned_poses1 = []
    aligned_poses2 = []
    for i in range(len(timestamps1)):
        nearest_timestamp2 = find_nearest_timestamp(timestamps1[i], timestamps2)
        idx1 = np.where(timestamps2 == nearest_timestamp2)[0][0]
        if(timestamps1[i] - nearest_timestamp2 < 0.2):
            aligned_poses1.append(poses1[i])
            aligned_poses2.append(poses2[idx1])
    aligned_poses1 = np.array(aligned_poses1)
    aligned_poses2 = np.array(aligned_poses2)
    print(f"aligned_poses1: {aligned_poses1.shape}, aligned_poses2: {aligned_poses2.shape}")
    T_mat = rigid_transform_3D(aligned_poses1[::10,0:3], aligned_poses2[::10,0:3]) ## B = (T_mat @ A.T).T
    print(f"T_mat:\n {T_mat}")
    ## save (T_mat @ pose1.T).T as txt
    trans_pose1 = []
    for i in range(len(poses1)):
        pose_mat = covert_tumpose_to_matrix(poses1[i])
        pose_mat = T_mat @ pose_mat
        pose = convert_matrix_to_tumpose(pose_mat)
        trans_pose1.append([timestamps1[i], pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]])
    trans_pose1 = np.array(trans_pose1)
    ## save timestamps1 and trans_pose1 as tum format
    root_dir = os.path.dirname(trajfile1)
    np.savetxt(os.path.join(root_dir, 'aligned_traj.txt'), trans_pose1, fmt='%f %f %f %f %f %f %f %f')
    return T_mat

def traj_transform(trajfile, outfile, transform_matrix):
    ## read trajfile, tum format
    timestamps, poses = read_tum(trajfile)
    ## transform traj to dbcenters by init_tranform
    traj = []
    count = 0
    for pose in poses:
        pose_convert = convert_matrix_to_tumpose(transform_matrix @ covert_tumpose_to_matrix(pose))
        traj.append([timestamps[count], pose_convert[0], pose_convert[1], pose_convert[2], pose_convert[3], pose_convert[4], pose_convert[5], pose_convert[6]])
        count+=1
    traj = np.array(traj)
    ## save traj.txt
    np.savetxt(outfile, traj, fmt='%f %f %f %f %f %f %f %f')

def close_err_estimate(bagfile, pcd_topic, imu_topic, pcddir, tfirst=-1, tlast=-1):
    ## read first and last pointcloud scan with imu in bagfile.
    bag = rosbag.Bag(bagfile)
    first_pointcloud2_msg = None
    last_pointcloud2_msg = None
    imu_msg = None
    first_scan_t = 0
    last_scan_t = 0
    if tfirst==-1 or tlast==-1:
        for topic, msg, t in bag.read_messages(topics=[pcd_topic, imu_topic]):
            if topic == pcd_topic and imu_msg is not None:
                last_pointcloud2_msg = msg
                last_scan_t = t.to_sec()
            elif topic == imu_topic:
                imu_msg = msg
            if last_pointcloud2_msg is not None and imu_msg is not None and first_pointcloud2_msg is None:
                first_pointcloud2_msg = last_pointcloud2_msg
                first_scan_t = t.to_sec()
    else:
        for topic, msg, t in bag.read_messages(topics=[pcd_topic, imu_topic]):
            if topic == pcd_topic and imu_msg is not None and t.to_sec() > tfirst and first_pointcloud2_msg is None:
                first_pointcloud2_msg = msg
                first_scan_t = t.to_sec()
            if topic == pcd_topic and imu_msg is not None and t.to_sec() > tlast and last_pointcloud2_msg is None:
                last_pointcloud2_msg = msg
                last_scan_t = t.to_sec()
            if topic == imu_topic:
                imu_msg = msg
    print("first_scan_t: ", first_scan_t)
    print("last_scan_t: ", last_scan_t)
    
    gen = pc2.read_points(first_pointcloud2_msg, skip_nans=True)
    int_data = list(gen)
    xyz = np.array([[0, 0, 0]])
    for x in int_data:
        xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
    out_pcd_first = o3d.geometry.PointCloud()
    out_pcd_first.points = o3d.utility.Vector3dVector(xyz)
    output_filename = os.path.join(pcddir, "first.pcd")
    o3d.io.write_point_cloud(output_filename, out_pcd_first)
    gen = pc2.read_points(last_pointcloud2_msg, skip_nans=True)
    int_data = list(gen)
    xyz = np.array([[0, 0, 0]])
    for x in int_data:
        xyz = np.append(xyz, [[x[0], x[1], x[2]]], axis=0)
    out_pcd_last = o3d.geometry.PointCloud()
    out_pcd_last.points = o3d.utility.Vector3dVector(xyz)
    output_filename = os.path.join(pcddir, "last.pcd")
    o3d.io.write_point_cloud(output_filename, out_pcd_last)

    ## regis out_pcd_first and out_pcd_last by icp.
    init_trans_mat = np.identity(4)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        out_pcd_first, out_pcd_last, 0.1, init_trans_mat,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    )
    lastL_T_firstL = reg_p2p.transformation
    print("lastL_T_firstL: ")
    print(lastL_T_firstL)

    out_pcd_first_trans = copy.deepcopy(out_pcd_first)
    out_pcd_first_trans.transform(lastL_T_firstL)
    o3d.io.write_point_cloud(output_filename, out_pcd_last)

    # mti_R_hesai = [-0.099438, -0.992995, -0.063813,
    #                 0.994106, -0.101923,  0.036942,
    #                -0.043187, -0.059763,  0.997278]
    # mti_R_hesai = np.array(mti_R_hesai).reshape(3, 3)
    # mti_t_hesai = [-0.012713, -0.001295, 0.184497]
    # mti_t_hesai = np.array(mti_t_hesai).reshape(3, 1)

    mti_T_hesai = [-0.099438, -0.992995, -0.063813, -0.012713,
                    0.994106, -0.101923,  0.036942, -0.001295,
                   -0.043187, -0.059763,  0.997278,  0.184497,
                           0,         0,         0,         1]
    mti_T_hesai = np.array(mti_T_hesai).reshape(4, 4)
    lastI_T_firstI = mti_T_hesai @ lastL_T_firstL @ np.linalg.inv(mti_T_hesai)
    print("lastI_T_firstI: ")
    print(lastI_T_firstI)

    

    



if __name__ == '__main__':
    # parser = argparse.ArgumentParser()
    # parser.add_argument('bagfile', type=str, help='rosbag file')
    # parser.add_argument('--topic', type=str, help='topic name', default='/velodyne_points')
    # parser.add_argument('--imu_topic', type=str, help='topic name', default='/zed2i/zed_node/imu/data')
    # parser.add_argument('--pcddir', type=str, help='pcd file directory')
    # args = parser.parse_args()

    # if not os.path.exists(args.pcddir):
    #     os.mkdir(args.pcddir)

    # extract_pcd_withimu(args.bagfile, args.topic, args.imu_topic, args.pcddir)
    # merge_pose_as_traj('/media/wbl/KESU/data/whu_tls_1030/project1/trans/absolute_pgo/')

    # bagfile = "/media/wbl/Elements/data/20240113/data1_aligned.bag"
    # pcd_topic = "/hesai/pandar"
    # imu_topic = "/imu/data"
    # pcddir = "/media/wbl/Elements/data/20240113/data1_aligned/"
    # if not os.path.exists(pcddir):
    #     os.mkdir(pcddir)
    # extract_pcd_withimu(bagfile, pcd_topic, imu_topic, pcddir)

    # init_trans_mat_file = "/media/wbl/Elements/data/20240113/data1_aligned/init_trans.txt"
    # init_trans_mat = np.loadtxt(init_trans_mat_file) # W_T_lidar0
    # W_T_lidar0 = init_trans_mat
    # print("init W_T_lidar0: ")
    # print(W_T_lidar0)
    # ## refine tramfrom matrix by icp
    # src_pcd_file = "/media/wbl/Elements/data/20240113/data1_aligned/1705133370653829949.pcd"
    # tgt_pcd_file = "/home/wbl/code/whu_tls_1030/pointbase/regis/3.pcd"
    # A_pcd = o3d.io.read_point_cloud(src_pcd_file)
    # B_pcd = o3d.io.read_point_cloud(tgt_pcd_file)
    # reg_p2p = o3d.pipelines.registration.registration_icp(
    #     A_pcd, B_pcd, 0.1, init_trans_mat,
    #     o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    #     o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
    # )
    # W_T_lidar0 = reg_p2p.transformation
    # print("icp refined W_T_lidar0: ")
    # print(W_T_lidar0)

    # ## convert to imu2w
    # mti_R_hesai = [-0.099438, -0.992995, -0.063813,
    #                 0.994106, -0.101923,  0.036942,
    #                 -0.043187, -0.059763,  0.997278]
    # hesai_R_mti = np.array(mti_R_hesai).reshape(3,3).T
    # lidar_T_imu = np.array([[hesai_R_mti[0,0], hesai_R_mti[0,1], hesai_R_mti[0,2], 0],
    #                         [hesai_R_mti[1,0], hesai_R_mti[1,1], hesai_R_mti[1,2], 0],
    #                         [hesai_R_mti[2,0], hesai_R_mti[2,1], hesai_R_mti[2,2], 0],
    #                         [0, 0, 0, 1]])
    # W_T_imu0 = W_T_lidar0 @ lidar_T_imu # W_T_imu
    # init_trans_mat = W_T_imu0

    # ## place_reloc_by_traj
    # traj_file = "/media/wbl/Elements/data/20240113/data1_aligned/pos_log.txt"
    # dbcenters_file = "/home/wbl/code/whu_tls_1030/pointbase/regis/centers.txt"
    # # init_trans_mat = np.array([[0.9999999999999998, -1.1102230246251565e-16, 0.0, 0.0],
    # #                              [1.1102230246251565e-16, 0.9999999999999998, 0.0, 0.0],
    # #                              [0.0, 0.0, 1.0, 0.0],
    # #                              [0.0, 0.0, 0.0, 1.0]])
    # place_reloc_by_traj(traj_file, dbcenters_file, init_trans_mat)

    # ## traj align
    # trajfile1 = "/home/wbl/code/process_data_tmp/data1_aligned/scan_states.txt"
    # trajfile2 = "/home/wbl/code/process_data_tmp/ars548_radar1_raw/bynavpose.txt"
    # T_mat = traj_align(trajfile1, trajfile2, ' ', ',')
    # ## traj transform
    # trajfile = "/home/wbl/code/whu_tls_1030/pointbase/regis/centers.txt"
    # outfile = "/home/wbl/code/whu_tls_1030/pointbase/regis/centers_aligned.txt"
    # traj_transform(trajfile, outfile, T_mat)

    parser = argparse.ArgumentParser()
    parser.add_argument('--datedatadirs', type=str, help='date data file', default='/media/wbl/Elements/data/20240113/')
    parser.add_argument('--seq_num', type=str, help='seq num', default='3')
    parser.add_argument('--dbcentersfile', type=str, help='db centers file', default='/home/wbl/code/whu_tls_1030/pointbase/regis/centers_aligned.txt')
    args = parser.parse_args()
    
    datedatadirs = args.datedatadirs
    traj_file = os.path.join(datedatadirs, 'ars548_radar' + args.seq_num + '_raw/bynavpose.txt')
    dbcenters_file = args.dbcentersfile
    init_trans_mat = np.identity(4)
    output_file = os.path.join(datedatadirs, 'data' + args.seq_num + '_aligned')

    # # ## place_reloc_by_traj
    # # traj_file = "/media/wbl/Elements/data/20240113/ars548_radar3_raw/bynavpose.txt"
    # # dbcenters_file = "/home/wbl/code/whu_tls_1030/pointbase/regis/centers_aligned.txt"
    # # init_trans_mat = np.identity(4)
    # clusters_cover_ts, t0 = place_reloc_by_traj(traj_file, dbcenters_file, init_trans_mat, split_format1=',', split_format2=' ')
    # ## save clusters_cover_ts as txt
    # if not os.path.exists(output_file):
    #     os.mkdir(output_file)
    # output_file = os.path.join(output_file, 'makeGT')
    # if os.path.exists(output_file):
    #     os.system(f"rm -r {output_file}")
    # if not os.path.exists(output_file):
    #     os.mkdir(output_file)
    # np.savetxt(os.path.join(output_file, 'GT_cover_ts.txt'), clusters_cover_ts, fmt='%f %f %f')

    # ## extract pcd from bagfile
    # bagfile = os.path.join(datedatadirs, 'data' + args.seq_num + '_aligned.bag')
    # pcd_topic = "/hesai/pandar"
    # imu_topic = "/imu/data"
    # pcddir = output_file
    # for i in range(len(clusters_cover_ts)):
    #     print(clusters_cover_ts[i][0])
    #     extract_pcd_withimu(bagfile, pcd_topic, imu_topic, pcddir, start_time=clusters_cover_ts[i][0]+t0)

    bagfile = os.path.join(datedatadirs, 'data' + args.seq_num + '_aligned.bag')
    pcd_topic = "/hesai/pandar"
    imu_topic = "/imu/data"
    pcddir = output_file
    close_err_estimate(bagfile, pcd_topic, imu_topic, pcddir, tfirst = 1706666998.298005342, tlast = 1706668214.119318247)
    
    
















    # merge_project1_2("/home/wbl/code/whu_tls_1030/project1/", "/home/wbl/code/whu_tls_1030/project2/", "/home/wbl/code/whu_tls_1030/pointbase/")
