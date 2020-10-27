#!/bin/bash

LIVETAGS_NAME="livetags"

while true
do
    read -s -p "Press enter key to starte a comment with time stamp... "
    now=$(date +"%Y-%m-%d-%H-%M-%S")
    echo ''
    echo ''
    echo $now
    read comment
    echo ''
    echo "[$now]#$comment#" >> $LIVETAGS_NAME
    echo 'Done.'

done

