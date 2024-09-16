#!/bin/bash

function sub() {
    local topic_name
    local file_name

    read -p "Type the topic_name: " topic_name
    read -p "Type the file_name.csv: " file_name

    # publish ros2 topic, press CTRL + C  to stop publishing
    ros2 topic pub  --rate 10 $topic_name geometry_msgs/msg/Pose

# save topics
    echo "$topic_name"  > "$file_name"
    nano $file_name

}

# Call the function
sub

# I add this line for Project-1 Step4 (Pull Request)
