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
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"

#include <opencv2/core/core.hpp>
#include "Converter.h"

#include "../../../include/System.h"
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

class ImageGrabber{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    void PublishPose(cv::Mat Tcw);  //
    void PublishOdom(cv::Mat Tcw);
    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
    ros::Publisher* pPosPub;        //
    ros::Publisher* pOdomPub;       //
    ros::Publisher* pQualityPub;
    tf::TransformBroadcaster* tfBroadCaster; //
    // ros::Publisher* pStatePub;
};

float Angle_x, Angle_y, Angle_z;
int camera2_reliability = 0;

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
        poseMSG.pose.position.z = twc.at<float>(1);
        poseMSG.pose.orientation.x = Angle_x;
        poseMSG.pose.orientation.y = Angle_y;
        poseMSG.pose.orientation.z = Angle_z;
        poseMSG.pose.orientation.w = q[3];
        poseMSG.header.frame_id = "VSLAM_Stereo";
        poseMSG.header.stamp = ros::Time::now();
        // cout << "PublishPose position.x = " << poseMSG.pose.position.x << endl;

        (pPosPub)->publish(poseMSG);
        // (pStatePub)->publish(State);
        //mlbLost.push_back(mState==LOST);
    }
}

void ImageGrabber::PublishOdom(cv::Mat Tcw){
  nav_msgs::Odometry OdomMSG;
  geometry_msgs::TransformStamped odom_trans;
  if(!Tcw.empty()){

      cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
      cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

      vector<float> q = ORB_SLAM2::Converter::toQuaternion(Rwc);

      OdomMSG.pose.pose.position.x = twc.at<float>(0);
      OdomMSG.pose.pose.position.y = twc.at<float>(2);
      OdomMSG.pose.pose.position.z = twc.at<float>(1);
      OdomMSG.pose.pose.orientation.x = q[0];
      OdomMSG.pose.pose.orientation.y = q[1];
      OdomMSG.pose.pose.orientation.z = q[2];
      OdomMSG.pose.pose.orientation.w = q[3];
      OdomMSG.header.frame_id = "odom";
      OdomMSG.header.stamp = ros::Time::now();
      OdomMSG.child_frame_id = "base_link";

      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";

      odom_trans.transform.translation.x = twc.at<float>(0);
      odom_trans.transform.translation.y = twc.at<float>(2);
      odom_trans.transform.translation.z = twc.at<float>(1);
      odom_trans.transform.rotation.x = q[0];
      odom_trans.transform.rotation.y = q[1];
      odom_trans.transform.rotation.z = q[2];
      odom_trans.transform.rotation.w = q[3];

      (tfBroadCaster)->sendTransform(odom_trans);
      (pOdomPub)->publish(OdomMSG);
  }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OS2_2");
    ros::start();

    if(argc != 6)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify path_to_map bOnlyTracking" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,2,true, bool(std::string(argv[5]) == "true"));
    /*******Load Map*********/
    std::string OnlyTracking = std::string(argv[5]);
    if (OnlyTracking == "true")
    {
        SLAM.LoadMap("map2.bin");
        std::cout << "---bOnlyTracking: " << OnlyTracking << std::endl;
    }
    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera2/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera2/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::Publisher PosPub = nh.advertise<geometry_msgs::PoseStamped>("ORB_SLAM2_2/pose", 5); // Pose publisher
        igb.pPosPub = &(PosPub); //
    ros::Publisher OdomPub = nh.advertise<nav_msgs::Odometry>("ORB_SLAM2_2/odom", 5); //Odometry publisher
        igb.pOdomPub = &(OdomPub); //
    ros::Publisher QualityPub = nh.advertise<std_msgs::Int32>("ORB_SLAM2_2/quality", 5); // Pose publisher
        igb.pQualityPub = &(QualityPub); //

    tf::TransformBroadcaster odom_broadcaster;
        igb.tfBroadCaster = &(odom_broadcaster);
    // ros::Publisher StatePub = nh.advertise<std_msgs::Int16>("ORB_SLAM2/Camera_State", 5); //Camera Lost Publisher
        // igb.pStatePub = &(StatePub); //state = 2 is lost


    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    /*******Save Map*********/
    if (OnlyTracking == "false")
        SLAM.SaveMap("./map2.bin");

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");


    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        cv::Mat Tcw= mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec(),2);  //
        camera2_reliability = mpSLAM->GetTheReliabilityOfTheCamera_system();
        //cout<<"Ros Reliability = "<<camera2_reliability<<"\n";
        PublishPose(Tcw); //
        PublishOdom(Tcw); //
        (pQualityPub)->publish(camera2_reliability);
    }
    else
    {
        cv::Mat Tcw= mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec(),2); //
        camera2_reliability = mpSLAM->GetTheReliabilityOfTheCamera_system();
        //cout<<"Ros Reliability = "<<camera2_reliability<<"\n";
        PublishPose(Tcw); //
        PublishOdom(Tcw);
        (pQualityPub)->publish(camera2_reliability);

    }

}
