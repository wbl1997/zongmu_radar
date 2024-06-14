#!/bin/bash

# ZMsrcdatapath="$1"
# OURsrcdatapath="$2"
ZMsrcdatapath="/media/wbl/KESU/data/zongmu_dataset/csv20240131/"
OURsrcdatapath="/media/wbl/Elements/data/20240131/"

analyze_script_path="/home/wbl/code/zongmu/zongmu_radar/"
slam_ws_path="/home/wbl/code/zongmu/iriom_ws/"

zongmu_csv=(
# csv01
csv02
# csv03
# csv04
# csv05
# csv06
)

our_bag=(
# data1_aligned
data2_aligned
# data3_aligned
# data4_aligned
# data5_aligned
# data6_aligned
)

## 1. zongmu data analyze
data_analyze() {
    source activate radar_env
    seqnames=("$@")
    for seq in "${seqnames[@]}"; do
        echo "Analyze data: $ZMsrcdatapath/$seq"
        python $analyze_script_path/scripts/corner_radar_preprocess.py \
            --datafile="$ZMsrcdatapath/$seq/" \
            --inifile="$analyze_script_path/radar_para_sda_0126.ini"
    done
}
data_analyze "${zongmu_csv[@]}"

# ## 2. CVM radar slam
# radarslam() {
#     cd $slam_ws_path
#     source devel/setup.bash
#     bagnames=("$@")
#     for seq in "${bagnames[@]}"; do
#         echo "Processing bag: $ZMsrcdatapath/$seq.bag"
#         roslaunch iriom mapping_radar_zongmu_car_cvm.launch \
#         bagfile:="$ZMsrcdatapath/$seq.bag" save_directory:="$ZMsrcdatapath/${seq}/"
#     done
# }
# radarslam "${zongmu_csv[@]}"

# ## 3. sync
# data_sync() {
#     source activate radar_env
#     args=("$@")
#     half_len=$(( ${#args[@]} / 2 ))
#     zongmu_bags=("${args[@]:0:$half_len}")
#     our_bags=("${args[@]:$half_len}")
#     for i in "${!zongmu_bags[@]}"; do
#         ZM_bag=${zongmu_bags[i]}
#         our_bag=${our_bags[i]}
#         echo "sync data: $ZMsrcdatapath/$ZM_bag"
#         python $analyze_script_path/scripts/timesync_bygyro.py \
#             --srcbagfile="$OURsrcdatapath/$our_bag.bag" \
#             --tumposefile="$ZMsrcdatapath/${ZM_bag}/pos_log.txt"
#     done
# }
# data_sync "${zongmu_csv[@]}" "${our_bag[@]}"

## 4. merge
data_merge() {
    source activate radar_env
    args=("$@")
    half_len=$(( ${#args[@]} / 2 ))
    zongmu_bags=("${args[@]:0:$half_len}")
    our_bags=("${args[@]:$half_len}")
    for i in "${!zongmu_bags[@]}"; do
        ZM_bag=${zongmu_bags[i]}
        our_bag=${our_bags[i]}
        python $analyze_script_path/scripts/data_merge_ZM.py \
            --ZMbagfile="$ZMsrcdatapath/$ZM_bag.bag" \
            --srcbagfile="$OURsrcdatapath/$our_bag.bag" \
            --dt=0
            # --dtfile="$ZMsrcdatapath/${ZM_bag}/dt.txt"
    done
}
data_merge "${zongmu_csv[@]}" "${our_bag[@]}"