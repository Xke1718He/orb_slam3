#ifndef SRC_GRABBER_H
#define SRC_GRABBER_H
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"../include/ImuTypes.h"
#include <termios.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace ORB_SLAM3
{
class ImageGrabber
{
public:
  explicit ImageGrabber(ORB_SLAM3::System* pSLAM, System::eSensor& sensor, string  settingPath,
               bool bClahe = false, bool do_rectify = false);
  void SyncWithImu();
  void DetectStopAndSaveMap();
private:
  void GrabImage(const sensor_msgs::ImageConstPtr& msg);
  void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);
  void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD);
  void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

  static cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

  void RunMonoInertial();
  void RunStereoInertial();
  void RunRGBDInertial();
  vector<ORB_SLAM3::IMU::Point> GetIMUData(double imTime);
  void SetStopFlag(bool flag);
  void RectifyImage();
  void ParseTopics();

  std::string mSettingPath;
  System::eSensor mSensor;

  queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf, imgDBuf;
  std::mutex mBufMutex;

  queue<sensor_msgs::ImuConstPtr> imuBuf;
  std::mutex mBufMutexIMU;

  ORB_SLAM3::System* mpSLAM;

  bool do_rectify;
  cv::Mat M1l,M2l,M1r,M2r;

  bool mbClahe;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));

  bool mbStop = false;
  std::mutex mbStopMutex;

  ros::NodeHandle nh_;
  ros::Subscriber subImu_;
  ros::Subscriber subImage_;
  std::string mLeftTopic;
  std::string mRightTopic;
  std::string mDepthTopic;
  std::string mIMUTopic;

  message_filters::Subscriber<sensor_msgs::Image> subDepthImg_;
  message_filters::Subscriber<sensor_msgs::Image> subLeftImg_;
  message_filters::Subscriber<sensor_msgs::Image> subRightImg_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> syncPolicy_;
  typedef message_filters::Synchronizer<syncPolicy_> Sync;
  boost::shared_ptr<Sync> sync_;
};
}
#endif//SRC_GRABBER_H
