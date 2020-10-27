#!/bin/bash

check_bag()
{
    echo "";
    echo "";
    rosrun kitcar_rosbag check.py "$1";
}

write_repo_info()
{
  echo $REPO_WITH_INFO
  if [ -d "$KITCAR_REPO_PATH/$REPO_WITH_INFO" ]; then
    echo "$REPO_WITH_INFO repository branch:">>rosbag_info.txt;
    (cd $KITCAR_REPO_PATH/$REPO_WITH_INFO && git symbolic-ref HEAD --short) >>rosbag_info.txt;
    echo "">>rosbag_info.txt;
    (cd $KITCAR_REPO_PATH/$REPO_WITH_INFO && git rev-list --format=medium --max-count=1 HEAD) >>rosbag_info.txt;
    echo "">>rosbag_info.txt;
  fi;
}

write_rosbag_info_and_diff()
{
	echo "rosbag_name=$1" >rosbag_info.txt;
	echo "time=$now">>rosbag_info.txt;
	echo "CAR_NAME=$CAR_NAME" >>rosbag_info.txt;
	echo "">>rosbag_info.txt;
  REPO_WITH_INFO=kitcar-ros
  write_repo_info
  REPO_WITH_INFO=kitcar-gazebo-simulation
  write_repo_info
}

write_camera_car_specs()
{
	#extract alternate yaml-file for camera and car_specs, which can be used for parameter tuning
	awk 'BEGIN{RS="(^|\n)[a-z_]+:"; print_next_line=0; ORS="";}
	print_next_line {print; print_next_line=0;}
	RT~/car_specs|camera/ {print RT; print_next_line=1;}
	END{print "\n";}' "params_$1.yaml" > "camera_car_specs_$1.yaml"
}

write_tag_template_file()
{
    cat > tags.txt <<EOF
== Junction ==
] 90 stop lines
] not 90 stop lines
] 90 give way lines
] not 90 give way lines
] Arrow
] abknickende vorfahrtstrasse

== Parking ==
] parallel parking
] perpendicualr parking

== Mixed ==
] obstacle

== Urban Area ==
] pedestrian island
] crosswalk
] no passing zone
] road cloasure

== Speed Limit ==
] start 10 Speedlimit
] start 20 Speedlimit
] start 30 Speedlimit
] start 40 Speedlimit
] start 50 Speedlimit
] start 60 Speedlimit
] start 70 Speedlimit
] start 80 Speedlimit
] end 10 Speedlimit
] end 20 Speedlimit
] end 30 Speedlimit
] end 40 Speedlimit
] end 50 Speedlimit
] end 60 Speedlimit
] end 70 Speedlimit
] end 80 Speedlimit
EOF

}

if [ -z "$KITCAR_REPO_PATH" ]
then
    KITCAR_REPO_PATH=$HOME;
fi

if [ $# -eq 0 ] ||  [ "$1" = '-h' ] || [ "$1" = '--help' ]
then
	tput setaf 2; echo 'Usage: sh record.sh -o PREFIX TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
	echo "or"
	echo 'Usage: sh record.sh -O NAME TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
	echo "or"
	echo 'Usage: sh record.sh TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'; tput sgr0
	echo ""
	echo "Create folder, dump parameters and record rosbag in this directory."
	echo ""
	echo 'ROSBAG_RECORD_OPTIONS:'
	rosbag record -h | sed '1,3 d'
	exit 0
elif [ "$1" = '-o' ]
then
	rosbag_dir=$2
	rosbag_prefix=$(echo "$rosbag_dir" | awk -F/ '{ if($NF!=""){ folder=$NF} else if(NF>1){ folder=$(NF-1)};  print folder; }')
	rosbag_dir=$(echo "$rosbag_dir" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
	rosbag_prefix="$rosbag_prefix""_"
	now=$(date +"%Y-%m-%d-%H-%M-%S")

	mkdir "$rosbag_dir""_""$now" &&
	(
		cd "$rosbag_dir""_""$now" && {
		write_rosbag_info_and_diff "$rosbag_dir$now"

		rosparam dump "params_$rosbag_prefix$now.yaml";
		write_camera_car_specs "$rosbag_prefix$now"
		write_tag_template_file
		shift;
		shift;

    # shellcheck disable=SC2064
		trap "{ check_bag $rosbag_prefix$now.bag; }" INT
		rosbag record -O "$rosbag_prefix$now" "$@";}
	)

elif [ $1 = '-O' ]
then
	rosbag_dir=$2
	rosbagname=$(echo $rosbag_dir | awk -F/ '{ if($NF!=""){ folder=$NF} else if(NF>1){ folder=$(NF-1)};  print folder; }')
	rosbag_dir=$(echo "$rosbag_dir" | awk -v h="$HOME" -F/ 'BEGIN {OFS="/"}{ if($1=="~"){$1=h};  print $0; }')
	now=$(date +"%Y-%m-%d-%H-%M-%S")

	mkdir "$rosbag_dir" &&

	(
		cd "$rosbag_dir" && {
		write_rosbag_info_and_diff "$rosbag_dir"

		rosparam dump "params_$rosbagname.yaml";
		write_camera_car_specs "$rosbagname"
		write_tag_template_file
		shift;
		shift;

    # shellcheck disable=SC2064
		trap "{ check_bag $rosbagname.bag ; }"  INT
		rosbag record -O "$rosbagname" "$@";}
	)

else
	for var in "$@"
	do
		if [ "$var" = '-o' ] || [ "$var" = '-O' ]
		then
			tput bold; tput setaf 1; echo "The name or prefix option has to be the first argument."
			echo ""
			tput setaf 2; echo 'Usage: sh record.sh -o PREFIX TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
			echo "or"
			echo 'Usage: sh record.sh -O NAME TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'
			echo "or"
			echo 'Usage: sh record.sh TOPIC1 [TOPIC2 TOPIC3 ...] [ROSBAG_RECORD_OPTIONS]'; tput sgr0
			echo ""
			echo "Create folder, dump parameters and record rosbag in this directory."
			echo ""
			echo 'ROSBAG_RECORD_OPTIONS:'
			rosbag record -h | sed '1,3 d'
			exit 1
		fi
	done

	now=$(date +"%Y-%m-%d-%H-%M-%S")


	mkdir "$now" &&

	(
		cd $now && {
		echo "rosbag_name=$now" >rosbag_info.txt;
		write_rosbag_info_and_diff "$now"

		rosparam dump params_$now".yaml";
		write_camera_car_specs "$now"
		write_tag_template_file

    # shellcheck disable=SC2064
		trap "{ check_bag $now.bag; }" INT
		rosbag record -O $now "$@";}
	)
fi

