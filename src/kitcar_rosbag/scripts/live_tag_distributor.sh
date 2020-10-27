#!/bin/bash

# some functions
writeTagToBag () {
    echo "Writeing $2 into $1"
    if [ ! -f $1 ]
    then
        touch ${1}
        echo -e "\e[33mWARNING:\e[39m There is no file ${1}. I created one, but the standard tags are now missing!"
    fi
    echo $2 >> $1
}


if [ -z ${LIVETAGS_NAME+x} ]; then echo "Environment variables are missing! Continue with caution!"; LIVETAGS_NAME="livetags"; fi

TAGS_NAME="tags.txt"
DELIMITER="-"

if [ $# -eq 0 ]
then
   echo -e "This script writes all TAGS saved in \e[32mlivetags\e[39m into the corresponding \e[33mtags\e[39m file for each rosbag
    PATH = path/to/folder/containing/all/bags
    PATH
    ├── rosbag_date1
    │   ├── rosbag_date1.yaml
    │   ├── rosbag_date1.bag
    │   ├── rosbag_info.txt
    │   ├── ros_diff
    │   ├── \e[33m${TAGS_NAME}\e[39m <-- The info will be written here
    ├── rosbag_date2
    .
    .
    ├── rosbag_dateN
    ├── \e[32m${LIVETAGS_NAME}\e[39m <-- The liveTag info inside PATH on the first level

    Usage: ./liveTagDistributer.sh PATH"
    exit 1
fi

BAGPATH="$1"

if [ ! -d "$1" ]
then
    echo -e "Directory \e[1m$1\e[0m DOES NOT exists."
    exit 1
fi

# append / if the last character is not a /
[[ $BAGPATH != */ ]] && BAGPATH="$BAGPATH"/

# read the livetags file and put each line into an array
declare -a all_live_tags
num_live_tags=0;
input="${BAGPATH}/${LIVETAGS_NAME}"
while IFS= read -r line
do
    all_live_tags[num_live_tags]="$line"
    ((num_live_tags+=1))
done < "$input"

bags=($BAGPATH*/)

# I do both arrays from behind ( ͡° ͜ʖ ͡°) to be able to compare the times since bag time is BEGIN and thus the livetag time must be higher/after the begin time.
current_live_tag_i=$(($num_live_tags-1));
num_bags=${#bags[@]}
current_bag_i=$((num_bags-1));

while [ "$current_bag_i" -gt "-1" ]
do
    rosbag_folder=${bags[$current_bag_i]}

    # This seperates the rosbag path XXXX2020-02-14-10-22-01/ and deletes the last character
    # I am too lazy to seperate the text and the year. I assume all live tags where done in the same year.
    IFS=$DELIMITER read -r -a rosback_times <<< "${rosbag_folder::-1}"
    tag_file="$rosbag_folder$TAGS_NAME"

    # as long as we are not @  zero we have some livetags to distribute
    while [ "$current_live_tag_i" -gt "-1" ]
    do
        # Do the very same with the next livetag
        # example: [2020-01-26-18-26-53]#ich bin ein test#
        current_live_tag=${all_live_tags[current_live_tag_i]}
        # first cut at ]

        IFS="]" read -r -a livetag_time <<< "${current_live_tag}"
        # now the first array element will contain: xxx[THE DATE

        IFS="[" read -r -a livetag_time2 <<< "${livetag_time[0]}"
        # now the second array element will contain: THE DATE

        # so we can again use delimiter to seperate the times
        IFS=$DELIMITER read -r -a livetag_times <<< "${livetag_time2[1]}"

        num_times_to_compare=5
        start_bag_time_index=$((${#rosback_times[@]}-$num_times_to_compare));
        start_tag_time_index=$((${#rosback_times[@]}-$num_times_to_compare));

        # Now compare the timestemps which are month-day-hour-minute-second
        use_next_bag=0
        for j in {0..4};
        do
            time_tag=${livetag_times[(($start_tag_time_index+$j))]}
            time_bag=${rosback_times[(($start_bag_time_index+$j))]}
            if [ "$time_tag" -gt "$time_bag" ]
            then
                use_next_bag=1
                # This livetag belongs to the current rosback, write it in

                writeTagToBag "$tag_file" "$current_live_tag"
                ((current_live_tag_i-=1))
                break
            fi
        done
        if [ $use_next_bag -eq "0" ]
        then
            # The timestemp on the livetag is junger than the bag timestamp. Thus we conclude that the current livetag must belong to an rosbag from before. So we break.
            ((current_bag_i-=1))
            break
        fi
    done
done

# some output
if [ "$current_live_tag_i" -gt "-1" ]
then
    echo "I am terrible sorry, but I was not able to write the following liveTags to any bag:"
    for((i=current_live_tag_i;i>=0;i--));
    do
	echo ${all_live_tags[i]}
    done
fi
