/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
// king

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int16.h"

#include <opencv2/core/core.hpp>
#include "Converter.h"

#include "../../../include/System.h"
#include "../../../include/Tracking.h"
#include "../../../include/FrameDrawer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <mutex>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
using namespace ORB_SLAM2;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    void PublishPose(cv::Mat Tcw);  //
    void PublishOdom(cv::Mat Tcw);

    ORB_SLAM2::System* mpSLAM;
    ros::Publisher* pPosPub;        //
    ros::Publisher* pOdomPub;       //
    tf::TransformBroadcaster* tfBroadCaster; //
    // ros::Publisher* pStatePub;
};

float Angle_x, Angle_y, Angle_z;
// int State;

void ImageGrabber::PublishPose(cv::Mat Tcw){
    // State = Camera_State;

    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty()){

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        //x-axis orientation
        double sinr_cosp = 2*(q[3]*q[0]+q[1]*q[2]);
        double cosr_cosp = 1-2*(q[0]*q[0]+q[1]*q[1]);
        Angle_x = atan2(sinr_cosp, cosr_cosp)*180/M_PI;

        //y-axis
        double sinr1_cosp = 2*(q[1]*q[3]-q[0]*q[2]);
        double cosr1_cosp = 1-2*(q[1]*q[1]+q[2]*q[2]);
  		  Angle_y = atan2(sinr1_cosp, cosr1_cosp)*180/M_PI;

        //z-axis orientation
        double siny_cosp = 2*(q[3]*q[2]+q[0]*q[1]);
        double cosy_cosp = 1-2*(q[1]*q[1]+q[2]*q[2]);
        Angle_z = atan2(siny_cosp, cosy_cosp)*180/M_PI;

        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(2);
        // poseMSG.pose.position.z = twc.at<float>(1);
        // poseMSG.pose.orientation.x = Angle_x;
        poseMSG.pose.orientation.y = Angle_y;
        // poseMSG.pose.orientation.z = Angle_z;
        // poseMSG.pose.orientation.w = q[3];
        // poseMSG.header.frame_id = "VSLAM_Stereo";
        poseMSG.header.stamp = ros::Time::now();
        // cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        (pPosPub)->publish(poseMSG);
        // (pStatePub)->publish(State);
        //mlbLost.push_back(mState==LOST);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 RGBD path_to_vocabulary path_to_settings path_to_map bOnlyTracking" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true, bool(std::string(argv[4]) == "true"));

    std::string OnlyTracking = std::string(argv[4]);
    if (OnlyTracking == "true"){
      SLAM.LoadMap("mapRGBD.bin");
      std::cout << "--bOnlyTracking: " << OnlyTracking <<std::endl;
    }

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));

    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2/pose", 5);
      igb.pPosPub = &(PosPub);
    // ros::Publisher StatePub = nh.advertise<std_msgs::Int16>("ORB_SLAM2/Camera_State", 5);
      // igb.pStatePub = &(StatePub);

    ros::spin();
    // test
    std::cout << "---In Shutdown---" << std::endl;

    // Stop all threads
    SLAM.Shutdown();

    //Save nap
    if (OnlyTracking == "false")
      SLAM.SaveMap("./mapRGBD.bin");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    PublishPose(Tcw);

}
