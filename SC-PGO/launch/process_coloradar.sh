#!/bin/bash
# example usage: ./SC-PGO/launch/process_coloradar.sh /media/pi/BackupPlus/jhuai/data/coloradar/rosbags /home/pi/Desktop/temp/coloradar

bagpath="$1"
outputpath="$2"

aspen=(aspen_run0
aspen_run1
aspen_run2
aspen_run3
aspen_run4
aspen_run5
aspen_run6
aspen_run7
aspen_run8
aspen_run9
aspen_run10
aspen_run11)

arpg=(arpg_lab_run0
arpg_lab_run1
arpg_lab_run2
arpg_lab_run3
arpg_lab_run4)

outdoors=(outdoors_run0
outdoors_run1
outdoors_run2
outdoors_run3
outdoors_run4
outdoors_run5
outdoors_run6
outdoors_run7
outdoors_run8
outdoors_run9)

longboard=(longboard_run0
longboard_run1
longboard_run2
longboard_run3
longboard_run4
longboard_run5
longboard_run6
longboard_run7)

edgar_army=(edgar_army_run1
edgar_army_run2
edgar_army_run3
edgar_army_run4
edgar_army_run5)

ec_hallways=(ec_hallways_run0
ec_hallways_run1
ec_hallways_run2
ec_hallways_run3
ec_hallways_run4)

edgar_classroom=(edgar_classroom_run0
edgar_classroom_run1
edgar_classroom_run2
edgar_classroom_run3
edgar_classroom_run4
edgar_classroom_run5)

lidarslam() {
cd /home/pi/Documents/lidar/fastlioslam_ws/
source devel/setup.bash
bagnames=("$@")
for bag in "${bagnames[@]}"; do
  echo "Processing bag: $bag"
  mkdir -p $outputpath/$bag
  roslaunch aloam_velodyne fastlio_ouster64_coloradar.launch \
       bagname:=$bagpath/$bag.bag save_directory:=$outputpath/$bag/
done
}

mergescans() {
cd /home/pi/Documents/lidar/fastlioslam_ws/src/FAST_LIO_SLAM/SC-PGO/utils/python
script=makeMergedMap.py
for bag in "${bagnames[@]}"; do
  echo "Merging scans for bag: $bag"
  python3 $script $outputpath/$bag/ --points_in_scan=50000 --node_skip=1 # --o3d_vis
done
}

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/Documents/slam_devel/lib
# lidarslam "${aspen[@]}"
# lidarslam "${arpg[@]}"
# lidarslam "${outdoors[@]}"
# lidarslam "${longboard[@]}"
# lidarslam "${edgar_army[@]}"
# lidarslam "${ec_hallways[@]}"
# lidarslam "${edgar_classroom[@]}"

# rerun for failed seqs
failed=(
edgar_army_run4
arpg_lab_run4
ec_hallways_run1
edgar_army_run1)

lidarslam "${edgar_classroom[@]}"

lidarslam "${failed[@]}"
