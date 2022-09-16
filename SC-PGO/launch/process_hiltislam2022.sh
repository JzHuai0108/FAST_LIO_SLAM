# example usage: SC-PGO/launch/process_hiltislam2022.sh /media/jhuai/SeagateData/jhuai/data/hiltislam2022 /media/jhuai/OldWin8OS/jhuai/hiltislam-results/fastlioslam

bagpath="$1"
outputpath="$2"

bagnames=(
exp01_construction_ground_level
exp02_construction_multilevel
exp03_construction_stairs
exp04_construction_upper_level
exp05_construction_upper_level_2
exp06_construction_upper_level_3
exp07_long_corridor
exp09_cupola
exp11_lower_gallery
exp15_attic_to_upper_gallery
exp21_outside_building
exp10_cupola_2
exp14_basement_2
exp16_attic_to_upper_gallery_2
exp18_corridor_lower_gallery_2
exp23_the_sheldonian_slam
)

lidarslam() {
cd /media/jhuai/docker/lidarslam/catkin_fastlio_slam/
source devel/setup.bash

for bag in "${bagnames[@]}"; do
  echo "Processing bag: $bag"
  mkdir -p $outputpath/$bag
  roslaunch aloam_velodyne fastlio_hesai32.launch \
      bagname:=$bagpath/$bag.bag save_directory:=$outputpath/$bag/
done
}

mergescans() {
cd /media/jhuai/docker/lidarslam/catkin_fastlio_slam/src/FAST_LIO_SLAM/SC-PGO/utils/python
script=makeMergedMap.py
for bag in "${bagnames[@]}"; do
  echo "Merging scans for bag: $bag"
  python3 $script $outputpath/$bag/ --points_in_scan=50000 --node_skip=1 # --o3d_vis
done
}

mergescans
