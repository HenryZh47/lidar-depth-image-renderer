#!/bin/bash
# cd "${0%/*}"

catkin build
if [ $? -ne 0 ]; then
    echo "Build failed, see output message above."
    exit $?
fi

helpFunction() {
    echo ""
    echo "Usage: $0 -bag <bag_file_path>"
    echo -e "\t-b Path to dataset in the form of ROS bag"
    echo -e "\t-t Duration of ROS bag to play"
    #echo -e "\t-c Description of what is parameterC"
    exit 1 # Exit script after printing help
}

while getopts "b:t:" opt; do
    case "$opt" in
    b) bagPath="$OPTARG" ;;
    t) bagDuration="$OPTARG" ;;
    ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
    esac
done

# Print helpFunction in case parameters are empty
if [ -z "$bagPath" ]; then
    echo "No ROS bag specified, use -b to specify one."
    helpFunction
fi

if [ -z "$bagDuration" ]; then
    bagDuration=10
fi

# Begin script in case all parameters are correct
echo "[Dataset]:"
echo "$bagPath"

source ${0%/*}/../../../devel/setup.bash
echo "[Source]:"
echo "${0%/*}/../../../devel/setup.bash"

nProcessors=$(nproc)
echo "[#Cores]:"
echo $nProcessors

# Benchmark Serial Version
echo "-----------------------SERIAL-----------------------"
roslaunch lidar_depth_renderer subt.launch log_file_path:=${PWD}/serial.csv &> /dev/null &
rosbag play --clock -s 0 -u $bagDuration $bagPath

kill $(jobs -p)
echo "Killed [$!]"

# Benchmark OpenMP Version
for ii in `seq 1 $nProcessors`
do
    echo "-----------------------OPENMP-----------------------"
    echo "Testing with $ii cores"
    export OMP_NUM_THREADS=$ii
    
    roslaunch lidar_depth_renderer subt_omp.launch log_file_path:=${PWD}/omp_${ii}.csv &> /dev/null &
    rosbag play --clock -s 0 -u $bagDuration $bagPath

    kill $(jobs -p)
    echo "Killed [$!]"
done

# Benchmark CUDA Version
echo "------------------------CUDA------------------------"
roslaunch lidar_depth_renderer subt_cuda.launch log_file_path:=${PWD}/cuda.csv &> /dev/null &
rosbag play --clock -s 0 -u $bagDuration $bagPath

kill $(jobs -p)
echo "Killed [$!]"

# echo "Press 'q' to exit"
# while :; do
#     read -n 1 k <&1
#     if [[ $k = q ]]; then
#         break
#     else
#         echo "Press 'q' to exit"
#     fi
# done
