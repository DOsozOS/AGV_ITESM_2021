# ORB SLAM 2 MODIFIED VERSION

This section presents the ORB SLAM 2 modification necessary to run the simulations.
Download the ORB SLAM 2 repository following the oficial instructions until the section 
"3. Building ORB-SLAM2" library and examples where raulmur GitHub indicates the cloning of the repository. Clone the ORB SLAM2 repository in the home foler (~) for facility. 

Official ORB SLAM2 repository:
https://github.com/raulmur/ORB_SLAM2

1 - Once you have cloned the official repository,  substitute the raulmur ~/ORB_SLAM2/src folder with the src folder located in this GitHub repository. (This instructions assume ORB SLAM2 is cloned in home)

2 - Then, enter to the ~/ORB_SLAM2/Examples/ and substitute the ROS folder with the provided folder in this GitHub repository.

3 - Download the required maps in order to run the ORB SLAM 2 in the simulation from the next mega link: If the link does not work please send an email to diegoosorio0000@gmail.com
https://mega.nz/folder/QhRGVRiL#Kekjtx2p9pVPKVZkqHAJUA

4 - Paste the downloaded files map1.bin and map2.bin on the directory ~/ORB_SLAM2/Examples/ROS/ORB_SLAM2

5 - Go back to the folder ~/ORB_SLAM2 and compile the main algorithm

    cd ORB_SLAM2
    chmod +x build.sh
    ./build.sh
6 - Compile the ROS examples

    chmod +x build_ros.sh
    ./build_ros.sh
7 - in order to run the ORB SLAM2 algorithm with the simulation, launch the simulation and then: (sustitute the username word with your Ubuntu username)

    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/username/ORB_SLAM2/Examples/ROS
    cd ~/ORB_SLAM2/Examples/ROS/ORB_SLAM2
    rosrun ORB_SLAM2 Stereo1 /home/username/ORB_SLAM2/Vocabulary/ORBvoc.txt    /home/username/ORB_SLAM2/Examples/ROS/ORB_SLAM2/Asus1.yaml false /home/username/ORB_SLAM2/Examples/ROS/ORB_SLAM2/src/maps true
    
