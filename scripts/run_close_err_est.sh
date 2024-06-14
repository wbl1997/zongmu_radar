#!/bin/bash
bagpath=/media/wbl/Elements/data/
dbcentersfile=/home/wbl/code/whu_tls_1030/pointbase/regis/centers_aligned.txt

bag20240103=(
# 20240103/1
# 20240103/2
# 20240103/3
# 20240103/4
# 20240103/5
# 20240103/6
)

bag20240113=(
# 20240113/1
# 20240113/2
# 20240113/3
# 20240113/4
# 20240113/5
)

bag20240115=(
# 20240115/1
# 20240115/2
# 20240115/3
# 20240115/4
)

bag20240116=(
# 20240116/2
# 20240116/3
# 20240116/4
# 20240116/5
)

bag20240116_eve=(
# 20240116_eve/1
# 20240116_eve/2
# 20240116_eve/3
# 20240116_eve/4
# 20240116_eve/5
)

bag20240123=(
# 20240123/1
# 20240123/2
# 20240123/3
)

bag20240131=(
20240131/2
# 20240131/3
# 20240131/5
)

process_all_data() {
bagnames=("$@")
for bag in "${bagnames[@]}"; do
  echo "Processing bag: $bagpath/$bag"
  datedatadirs=$bagpath/${bag%/*}/
  seqnum=${bag##*/}
  mkdir -p $datedatadirs/data$seqnum_aligned/
  python /home/wbl/code/zongmu/zongmu_radar/scripts/makeGroundtruth.py \
  --datedatadirs=$datedatadirs \
  --seq_num=$seqnum \
  --dbcentersfile=$dbcentersfile
done
}

# process_all_data "${bag20240103[@]}"
# process_all_data "${bag20240113[@]}"
# process_all_data "${bag20240115[@]}"
# process_all_data "${bag20240116[@]}"
# process_all_data "${bag20240116_eve[@]}"
# process_all_data "${bag20240123[@]}"
process_all_data "${bag20240131[@]}"


