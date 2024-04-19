# sh script for auto extract frames from multiple ros2 bags and store on respective folder
# -----

# Start and load environment
echo "Starting frames extraction from ros2 bags"
source /opt/ros/foxy/setup.bash
echo "$(rosversion -d) loaded correctly"

# For each bag, create corresponding frames folder
for bagfile in "./bags"/*; do
    echo "---- Processing bag $bagfile ----"
    bag_name=$(basename "$bagfile")
    folder_name="${bag_name%.*}"
    frames_path="./frames/$folder_name/"
    mkdir -p $frames_path

    # # ---
    # ros2 bag play "$bagfile" &
    # echo "This bag topics are:\n $(ros2 topic list)"
    # input "\n\nWhich topic should be subscribed: "
    # ---

    # Start python subscriber script
    #python3 ros2_bag2images.py /maria/color/image_raw $frames_path &
    python3 ros2_bag2images.py /marti/color/image_raw $frames_path &
    # Delay to let script start listening
    sleep 1
    # Run ros2 bag
    ros2 bag play "$bagfile"

    echo "Bagfile's frames $bagfile extracted succesfully!"
    pkill -9 -f ros2_bag2images.py
    sleep 2
done
