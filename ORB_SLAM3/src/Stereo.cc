/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


// 双目相机

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ros::NodeHandle nh;
    ros::Publisher pub_rgb, pub_depth, pub_tcw, pub_camerapath, pub_odom;
    size_t mcounter = 0;
    nav_msgs::Path camerapath;
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM),nh("~")
    {
        // 创建ROS发布话题
        pub_rgb= nh.advertise<sensor_msgs::Image> ("Left/Image", 10); 
	    pub_depth= nh.advertise<sensor_msgs::Image> ("Right/Image", 10); 
		pub_tcw= nh.advertise<geometry_msgs::PoseStamped> ("CameraPose", 10); 
		pub_odom= nh.advertise<nav_msgs::Odometry> ("Odometry", 10); 
		pub_camerapath= nh.advertise<nav_msgs::Path> ("Path", 10); 
    }

    void GrabStereo(const sensor_msgs::Image::ConstPtr msgLeft, const sensor_msgs::Image::ConstPtr msgRight);
    ORB_SLAM3::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "STEREO_PUB");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO,true);

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

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
	
	//SLAM.SaveKeyFrameTrajectoryKITTI("KeyFrameTrajectory_KITTI_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}


void ImageGrabber::GrabStereo(const sensor_msgs::Image::ConstPtr msgLeft,const sensor_msgs::Image::ConstPtr msgRight)
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
    
    
	bool  isKeyFrame =true;
	cv::Mat Tcw;
	
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        Tcw=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
		Tcw=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
        // Tcw=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec(),isKeyFrame);
    }
     
     if (!Tcw.empty())
	{
        cv::Mat Twc =Tcw.inv();
        cv::Mat RWC= Twc.rowRange(0,3).colRange(0,3);  
        cv::Mat tWC=  Twc.rowRange(0,3).col(3);
        //cv::Mat TWC=orbslam->mpTracker->mCurrentFrame.mTcw.inv();  
        //cv::Mat RWC= Tcw.rowRange(0,3).colRange(0,3).t();//Tcw.rowRange(0,3).colRange(0,3);  
        //cv::Mat tWC=  -RWC*Tcw.rowRange(0,3).col(3);//Tcw.rowRange(0,3).col(3);
        
        Eigen::Matrix<double,3,3> eigMat ;
        eigMat <<RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
                        RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
                        RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2);
        Eigen::Quaterniond q(eigMat);
 
        geometry_msgs::PoseStamped tcw_msg; 					
        tcw_msg.pose.position.x=tWC.at<float>(0);
        tcw_msg.pose.position.y=tWC.at<float>(1);			 
        tcw_msg.pose.position.z=tWC.at<float>(2);
				 
        tcw_msg.pose.orientation.x=q.x();
        tcw_msg.pose.orientation.y=q.y();
        tcw_msg.pose.orientation.z=q.z();
        tcw_msg.pose.orientation.w=q.w();
				 
        // tf::Matrix3x3 M(RWC.at<float>(0,0),RWC.at<float>(0,1),RWC.at<float>(0,2),
        //                 RWC.at<float>(1,0),RWC.at<float>(1,1),RWC.at<float>(1,2),
        //                 RWC.at<float>(2,0),RWC.at<float>(2,1),RWC.at<float>(2,2));
        // tf::Vector3 V(tWC.at<float>(0), tWC.at<float>(1), tWC.at<float>(2));
        
        // tf::Quaternion q;
        // M.getRotation(q);
        
        // tf::Pose tf_pose(q,V);
        
        // double roll,pitch,yaw;
        // M.getRPY(roll,pitch,yaw);
        // cout<<"roll: "<<roll<<"  pitch: "<<pitch<<"  yaw: "<<yaw;
        // cout<<"    t: "<<tWC.at<float>(0)<<"   "<<tWC.at<float>(1)<<"    "<<tWC.at<float>(2)<<endl;
        
        // if(roll == 0 || pitch==0 || yaw==0)
        // return ;
        // ------
				  
        std_msgs::Header header ;
        header.stamp =msgLeft->header.stamp;
        header.seq = msgLeft->header.seq;
        header.frame_id="world";
        //cout<<"depth type: "<< depth. type()<<endl;
        
        sensor_msgs::Image::ConstPtr rgb_msg = msgLeft;
        sensor_msgs::Image::ConstPtr depth_msg=msgRight;
				  
        //geometry_msgs::PoseStamped tcw_msg; 
        tcw_msg.header=header;
        //tf::poseTFToMsg(tf_pose, tcw_msg.pose);
				 
				 
 
        // odometry information
        nav_msgs::Odometry odom_msg;
        odom_msg.pose.pose.position.x=tWC.at<float>(0);
        odom_msg.pose.pose.position.y=tWC.at<float>(1);			 
        odom_msg.pose.pose.position.z=tWC.at<float>(2);
				 
        odom_msg.pose.pose.orientation.x=q.x();
        odom_msg.pose.pose.orientation.y=q.y();
        odom_msg.pose.pose.orientation.z=q.z();
        odom_msg.pose.pose.orientation.w=q.w();
        
        odom_msg.header=header;
        odom_msg.child_frame_id="base_link"; 

        // 发布TF 变换
        // static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
        // geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
        // geometry_msgs::Quaternion odom_quat;   //四元数变量

        // //里程计的偏航角需要转换成四元数才能发布
        // odom_quat = tf::createQuaternionMsgFromRollPitchYaw ( roll,  pitch,  yaw);
        // //载入坐标（tf）变换时间戳
        // odom_trans.header.stamp = msgLeft->header.stamp;
        // odom_trans.header.seq = msgLeft->header.seq;
        // //发布坐标变换的父子坐标系
        // odom_trans.header.frame_id = "odom";     
        // odom_trans.child_frame_id = "camera";       
        // //tf位置数据：x,y,z,方向
        // odom_trans.transform.translation.x = tWC.at<float>(0);
        // odom_trans.transform.translation.y = tWC.at<float>(1);
        // odom_trans.transform.translation.z = tWC.at<float>(2);
        // odom_trans.transform.rotation = odom_quat;        
        // //发布tf坐标变化
        // odom_broadcaster.sendTransform(odom_trans);
				  
				
				  
        camerapath.header =header;
        camerapath.poses.push_back(tcw_msg);
        //Tcw位姿信息
        pub_tcw.publish(tcw_msg);	                
        pub_odom.publish(odom_msg);	  
        //相机轨迹
        pub_camerapath.publish(camerapath);  
        if( isKeyFrame)
        {
            pub_rgb.publish(rgb_msg);
            pub_depth.publish(depth_msg);	
        }
	}
	else
	{
	  cout<<"Twc is empty ..."<<endl;
	}

}
