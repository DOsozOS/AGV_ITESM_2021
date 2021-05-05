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
#include "std_msgs/Int16.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>
#include "Converter.h"

#include"../../../include/System.h"
#include "../../../include/Tracking.h"
#include "../../../include/FrameDrawer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<mutex>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
using namespace std;
using namespace ORB_SLAM2;

double Angle_x = 0.0;
double Angle_y = 0.0;
double Angle_z = 0.0;
// int State;

class ImageGrabber{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    void PublishPose(cv::Mat Tcw);  //
    ORB_SLAM2::System* mpSLAM;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher* pPosPub;        //
    // ros::Publisher* pStatePub;
};



void ImageGrabber::PublishPose(cv::Mat Tcw){
    // State = Camera_State;
    //std::cout << "ESTADO ACTUAL: " << State << std::endl;

    geometry_msgs::PoseStamped poseMSG;
    if(!Tcw.empty()){

        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

        //x-axis orientation
        double sinr_cosp = 2*(q[3]*q[0]+q[1]*q[2]);
        double cosr_cosp = 1-2*(q[0]*q[0]+q[1]*q[1]);
        Angle_x = atan2(sinr_cosp, cosr_cosp)*180/M_PI;

        //y-axis orientation
		    double sinr1_cosp = 2*(q[1]*q[3]-q[0]*q[2]);
        double cosr1_cosp = 1-2*(q[1]*q[1]+q[2]*q[2]);
		    Angle_y = atan2(sinr1_cosp, cosr1_cosp)*180/M_PI;

        //z-axis orientation
        double siny_cosp = 2*(q[3]*q[2]+q[0]*q[1]);
        double cosy_cosp = 1-2*(q[1]*q[1]+q[2]*q[2]);
        Angle_z = atan2(siny_cosp, cosy_cosp)*180/M_PI;

      /*
        poseMSG.pose.orientation.x = q[0];
        poseMSG.pose.orientation.y = q[1];
        poseMSG.pose.orientation.z = q[2];
        poseMSG.pose.orientation.w = q[3];
      */

        /*
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);
            tf::Matrix3x3 M(Rwc.at<float>(0,0),Rwc.at<float>(0,1),Rwc.at<float>(0,2),
                            Rwc.at<float>(1,0),Rwc.at<float>(1,1),Rwc.at<float>(1,2),
                            Rwc.at<float>(2,0),Rwc.at<float>(2,1),Rwc.at<float>(2,2));
            tf::Vector3 V(twc.at<float>(0), twc.at<float>(1), twc.at<float>(2));

            tf::Transform tfTcw(M,V);

            //mTfBr.sendTransform(tf::StampedTransform(tfTcw,ros::Time::now(), "ORB_SLAM/World", "ORB_SLAM/Camera"));
        */
        poseMSG.pose.position.x = twc.at<float>(0);
        poseMSG.pose.position.y = twc.at<float>(2);
        poseMSG.pose.position.z = twc.at<float>(1);
        poseMSG.pose.orientation.x = Angle_x;
        poseMSG.pose.orientation.y = Angle_y;
        poseMSG.pose.orientation.z = Angle_z;
        //poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "VSLAM_Mono";
        poseMSG.header.stamp = ros::Time::now();
        //cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        (pPosPub)->publish(poseMSG);
        // (pStatePub)->publish(State);
        //mlbLost.push_back(mState==LOST);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    //bool bReuseMap = false;  //
    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings path_to_map bOnlyTracking" << endl;
        ros::shutdown();
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true, bool(std::string(argv[5]) == "true"));  //
    /*******Load Map*********/
    std::string OnlyTracking = std::string(argv[5]);
    if (OnlyTracking == "true"){
        SLAM.LoadMap("map_mono.bin");
        std::cout << "---bOnlyTracking: " << OnlyTracking << std::endl;
    }
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    //ros::Subscriber sub = nodeHandler.subscribe("/zed/zed_node/left/image_rect_color", 1, &ImageGrabber::GrabImage,&igb);
    ros::Subscriber sub = nodeHandler.subscribe("usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
    ros::Publisher PosPub = nodeHandler.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2/pose", 5); //
    igb.pPosPub = &(PosPub); //
    // ros::Publisher StatePub = nodeHandler.advertise<std_msgs::Int16>("ORB_SLAM2/Camera_State", 5);
    // igb.pStatePub = &(StatePub);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save map                            //
    if (OnlyTracking == "false")
        SLAM.SaveMap("./map_mono.bin"); //

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat Tcw= mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec()); //
    PublishPose(Tcw); //
}
