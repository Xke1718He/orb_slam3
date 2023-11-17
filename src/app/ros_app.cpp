#include"System.h"
#include "Grabber.h"

int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "ORB_SLAM3");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
  bool bEqual = false;
  bool bRect = false;
  if(argc < 3 || argc > 5)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 ORB_SLAM3 path_to_vocabulary path_to_settings [do_equalize] [do_rectify]" << endl;
    ros::shutdown();
    return -1;
  }

  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }
  if(argc==5)
  {
    std::string sbRect(argv[4]);
    if(sbRect == "true")
      bRect = true;
  }

  ORB_SLAM3::System::eSensor sensor;
  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
    LOG(ERROR) << "ERROR: Wrong path to settings";
    return -1;
  }
  cv::FileNode node = fsSettings["Mode.Sensor"];
  if(!node.empty() && node.isInt())
  {
    sensor = static_cast<ORB_SLAM3::System::eSensor>(node.operator int());
    if(sensor==ORB_SLAM3::System::MONOCULAR)
      LOG(INFO) << "Monocular";
    else if(sensor==ORB_SLAM3::System::STEREO)
      LOG(INFO) << "Stereo";
    else if(sensor==ORB_SLAM3::System::RGBD)
      LOG(INFO) << "RGB-D";
    else if(sensor==ORB_SLAM3::System::IMU_MONOCULAR)
      LOG(INFO) << "Monocular-Inertial";
    else if(sensor==ORB_SLAM3::System::IMU_STEREO)
      LOG(INFO) << "Stereo-Inertial";
    else if(sensor==ORB_SLAM3::System::IMU_RGBD)
      LOG(INFO) << "RGB-D-Inertial";
  }
  else
  {
    LOG(ERROR) << "Mode.Sensor parameter doesn't exist or is not an integer";
    return -1;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], sensor, false);
  ORB_SLAM3::ImageGrabber igb(&SLAM, sensor, argv[2], bEqual, bRect);

  std::thread sync_thread(&ORB_SLAM3::ImageGrabber::SyncWithImu,&igb);
  std::thread stop_thread(&ORB_SLAM3::ImageGrabber::DetectStopAndSaveMap, &igb);

  ros::spin();
  return 0;
}
