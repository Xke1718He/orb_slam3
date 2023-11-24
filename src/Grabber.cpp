#include "Grabber.h"

#include <utility>
namespace ORB_SLAM3
{
ImageGrabber::ImageGrabber(ORB_SLAM3::System *pSLAM, System::eSensor& sensor,
                           string  settingPath, bool bClahe, bool do_rectify)
    :mSettingPath(std::move(settingPath)), mSensor(sensor), mpSLAM(pSLAM), do_rectify(do_rectify), mbClahe(bClahe)
{
  ParseTopics();

  if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_RGBD || mSensor == ORB_SLAM3::System::IMU_STEREO)
  {
    subImu_ = nh_.subscribe(mIMUTopic, 1000, &ImageGrabber::GrabImu, this);
  }

  if (mSensor == ORB_SLAM3::System::RGBD || mSensor == ORB_SLAM3::System::IMU_RGBD)
  {
    subDepthImg_.subscribe(nh_, mDepthTopic, 100);
    subLeftImg_.subscribe(nh_, mLeftTopic, 100);
    sync_.reset(new Sync(syncPolicy_(10), subLeftImg_, subDepthImg_));
    sync_->registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this, _1, _2));
  }

  if (mSensor == ORB_SLAM3::System::STEREO || mSensor == ORB_SLAM3::System::IMU_STEREO)
  {
    subLeftImg_.subscribe(nh_, mLeftTopic, 100);
    subRightImg_.subscribe(nh_, mRightTopic, 100);
    sync_.reset(new Sync(syncPolicy_(10), subLeftImg_, subRightImg_));
    sync_->registerCallback(boost::bind(&ImageGrabber::GrabStereo, this, _1, _2));
  }

  if (mSensor == ORB_SLAM3::System::MONOCULAR || mSensor == ORB_SLAM3::System::IMU_MONOCULAR)
  {
    subImage_ = nh_.subscribe(mLeftTopic, 100, &ImageGrabber::GrabImage,this);
  }

  if (do_rectify)
  {
    RectifyImage();
  }
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  return cv_ptr->image.clone();
}

void ImageGrabber::SyncWithImu()
{
  while(!mbStop)
  {
    cv::Mat imLeft, imRight, imDepth;
    double tIm = 0;
    switch (mSensor)
    {
      case ORB_SLAM3::System::MONOCULAR:
        if(imgLeftBuf.empty())
        {
          continue;
        }

        {
          this->mBufMutex.lock();
          imLeft = GetImage(imgLeftBuf.front());
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          imgLeftBuf.pop();
          this->mBufMutex.unlock();
        }

        //tum
        if (imLeft.type() == 2)
        {
          imLeft /= 257;
          imLeft.convertTo(imLeft, CV_8UC1);
        }

        mpSLAM->TrackMonocular(imLeft,tIm);
        break;
      case ORB_SLAM3::System::IMU_MONOCULAR:
        if (!imgLeftBuf.empty() && !imuBuf.empty())
        {
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          if (tIm > imuBuf.back()->header.stamp.toSec())
            continue;

          RunMonoInertial();
        }
        break;
      case ORB_SLAM3::System::RGBD:
        if(imgLeftBuf.empty())
        {
          continue;
        }

        {
          this->mBufMutex.lock();
          imLeft = GetImage(imgLeftBuf.front());
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          imgLeftBuf.pop();
          imDepth = GetImage(imgDBuf.front());
          imgDBuf.pop();
          this->mBufMutex.unlock();
        }
        mpSLAM->TrackRGBD(imLeft, imDepth, tIm);
        break;
      case ORB_SLAM3::System::IMU_RGBD:
        if (!imgLeftBuf.empty() && !imuBuf.empty())
        {
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          if (tIm > imuBuf.back()->header.stamp.toSec())
            continue;

          RunRGBDInertial();
        }
        break;
      case ORB_SLAM3::System::STEREO:
        if(imgLeftBuf.empty())
        {
          continue;
        }

        {
          this->mBufMutex.lock();
          imLeft = GetImage(imgLeftBuf.front());
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          imgLeftBuf.pop();

          imRight = GetImage(imgRightBuf.front());
          imgRightBuf.pop();
          this->mBufMutex.unlock();
        }

        //tum
        if (imLeft.type() == 2)
        {
          imLeft /= 257;
          imLeft.convertTo(imLeft, CV_8UC1);

          imRight /= 257;
          imRight.convertTo(imRight, CV_8UC1);
        }

        mpSLAM->TrackStereo(imLeft, imRight, tIm);
        break;
      case ORB_SLAM3::System::IMU_STEREO:
        if (!imgLeftBuf.empty() && !imuBuf.empty())
        {
          tIm = imgLeftBuf.front()->header.stamp.toSec();
          if (tIm > imuBuf.back()->header.stamp.toSec())
            continue;

          RunStereoInertial();
        }
        break;
    }
    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
  ros::shutdown();
}

void ImageGrabber::DetectStopAndSaveMap()
{
  struct termios new_settings;
  struct termios stored_settings;
  while (true)
  {
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 20;
    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0,TCSANOW,&new_settings);

    int tmp = getchar();

    if(10 == tmp)
    {
      LOG(INFO) << "detect a key value:" << tmp;

      mpSLAM->Shutdown();
      SetStopFlag(true);
      break;
    }

    tcsetattr(0,TCSANOW,&stored_settings);
  }
}

void ImageGrabber::SetStopFlag(bool flag)
{
  std::unique_lock<std::mutex> lock(mbStopMutex);
  mbStop = flag;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
  mBufMutex.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(msgLeft);

  if (!imgRightBuf.empty())
    imgRightBuf.pop();
  imgRightBuf.push(msgRight);
  mBufMutex.unlock();
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
  mBufMutex.lock();

  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(msgRGB);

  if (!imgDBuf.empty())
    imgDBuf.pop();
  imgDBuf.push(msgD);

  mBufMutex.unlock();
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutex.lock();
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();
  imgLeftBuf.push(img_msg);
  mBufMutex.unlock();
}

void ImageGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutexIMU.lock();
  imuBuf.push(imu_msg);
  mBufMutexIMU.unlock();
}

void ImageGrabber::RunMonoInertial()
{
  cv::Mat im;
  const double tIm = imgLeftBuf.front()->header.stamp.toSec();

  {
    this->mBufMutex.lock();
    im = GetImage(imgLeftBuf.front());
    imgLeftBuf.pop();
    this->mBufMutex.unlock();
  }

  vector<ORB_SLAM3::IMU::Point> vImuMeas = GetIMUData(tIm);

  if(mbClahe)
    mClahe->apply(im,im);
  //tum
  if (im.type() == 2)
  {
    im /= 257;
    im.convertTo(im, CV_8UC1);
  }
  mpSLAM->TrackMonocular(im,tIm,vImuMeas);
}

void ImageGrabber::RunStereoInertial()
{
  cv::Mat imLeft, imRight;
  const double tIm = imgLeftBuf.front()->header.stamp.toSec();

  this->mBufMutex.lock();
  imLeft = GetImage(imgLeftBuf.front());
  imgLeftBuf.pop();
  imRight = GetImage(imgRightBuf.front());
  imgRightBuf.pop();
  this->mBufMutex.unlock();

  //tum
  if (imLeft.type() == 2)
  {
    imLeft /= 257;
    imLeft.convertTo(imLeft, CV_8UC1);

    imRight /= 257;
    imRight.convertTo(imRight, CV_8UC1);
  }

  vector<ORB_SLAM3::IMU::Point> vImuMeas = GetIMUData(tIm);
  if(mbClahe)
  {
    mClahe->apply(imLeft, imLeft);
    mClahe->apply(imRight, imRight);
  }

  if(do_rectify)
  {
    cv::remap(imLeft,imLeft,M1l,M2l,cv::INTER_LINEAR);
    cv::remap(imRight,imRight,M1r,M2r,cv::INTER_LINEAR);
  }

  Sophus::SE3f Tcw = mpSLAM->TrackStereo(imLeft, imRight, tIm, vImuMeas);
}

void ImageGrabber::RunRGBDInertial()
{
  cv::Mat im, depth;
  const double tIm = imgLeftBuf.front()->header.stamp.toSec();

  this->mBufMutex.lock();
  im = GetImage(imgLeftBuf.front());
  imgLeftBuf.pop();
  depth = GetImage(imgDBuf.front());
  imgDBuf.pop();
  this->mBufMutex.unlock();

  vector<ORB_SLAM3::IMU::Point> vImuMeas = GetIMUData(tIm);
  Sophus::SE3f Tcw = mpSLAM->TrackRGBD(im, depth, tIm, vImuMeas);
}

vector<ORB_SLAM3::IMU::Point> ImageGrabber::GetIMUData(double imTime)
{
  vector<ORB_SLAM3::IMU::Point> vImuMeas;
  mBufMutexIMU.lock();
  if (!imuBuf.empty())
  {
    // Load imu measurements from buffer
    while (!imuBuf.empty() && imuBuf.front()->header.stamp.toSec() <= imTime)
    {
      double t = imuBuf.front()->header.stamp.toSec();
      cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
      cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
      vImuMeas.emplace_back(acc, gyr, t);
      imuBuf.pop();
    }
  }
  mBufMutexIMU.unlock();
  return vImuMeas;
}
void ImageGrabber::RectifyImage()
{
  cv::FileStorage fsSettings(mSettingPath, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    LOG(ERROR) << "ERROR: Wrong path to settings";
    return;
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
    LOG(ERROR) << "ERROR: Calibration parameters to rectify stereo are missing!";
    return;
  }

  cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0,3).colRange(0,3), cv::Size(cols_l,rows_l), CV_32F, M1l, M2l);
  cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0,3).colRange(0,3), cv::Size(cols_r,rows_r), CV_32F, M1r, M2r);
}
void ImageGrabber::ParseTopics()
{
  cv::FileStorage fsSettings(mSettingPath, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    LOG(ERROR) << "ERROR: Wrong path to settings";
    return;
  }

  fsSettings["Topic.Left"] >> mLeftTopic;
  LOG(INFO) << "Left Topic: " << mLeftTopic;
  if (mSensor == ORB_SLAM3::System::STEREO || mSensor == ORB_SLAM3::System::IMU_STEREO)
  {
    fsSettings["Topic.Right"] >> mRightTopic;
    LOG(INFO) << "Right Topic: " << mRightTopic;
  }

  if (mSensor == ORB_SLAM3::System::RGBD || mSensor == ORB_SLAM3::System::IMU_RGBD)
  {
    fsSettings["Topic.Depth"] >> mDepthTopic;
    LOG(INFO) << "Depth Topic: " << mDepthTopic;
  }

  if (mSensor == ORB_SLAM3::System::IMU_MONOCULAR || mSensor == ORB_SLAM3::System::IMU_STEREO
      || mSensor == ORB_SLAM3::System::IMU_RGBD)
  {
    fsSettings["Topic.IMU"] >> mIMUTopic;
    LOG(INFO) << "IMU Topic: " << mIMUTopic;
  }
}
}