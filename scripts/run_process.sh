# python scripts/corner_radar_preprocess.py \
# --datafile="/media/wbl/KESU/data/zongmu_dataset/csv20240103/csv06/" \
# --inifile="/home/wbl/code/zongmu/zongmu_radar/radar_para_sda.ini"

# python scripts/data_merge_ZM.py \
# --ZMbagfile="/media/wbl/KESU/data/zongmu_dataset/csv20240103/csv06.bag" \
# --srcbagfile="/media/wbl/KESU/data/zongmu_dataset/20240103/data6_aligned.bag" \
# --dt=-37.2343738

analyze_script_path="/home/wbl/code/zongmu/zongmu_radar/"
ZMsrcdatapath="/media/wbl/KESU/data/zongmu_dataset/csv20240123/csv03"
data_analyze() {
    source activate radar_env
    python $analyze_script_path/scripts/corner_radar_preprocess.py \
        --datafile="$ZMsrcdatapath/" \
        --inifile="$analyze_script_path/radar_para_sda.ini"
}
data_analyze