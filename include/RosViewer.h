#ifndef ROS_VIEWER_H
#define ROS_VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"

//#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>  


#include <mutex>

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Atlas;

class ROSViewer
{
public:
  ROSViewer(System* pSystem, Tracking* pTracker, Atlas* pAtlas, FrameDrawer *pFrameDrawer);

    void Run();
    void RequestFinish();
    void RequestStop();
    bool isFinished();
    bool isStopped();
    void Release();
    void SetCurrentCameraPoseAndTime(const Sophus::SE3f &Tcw, double time);
    void SetLoopFrame(KeyFrame* pCurrentFrame, KeyFrame* pLoopMatchedKF);
    
private:

    bool Stop();
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    Tracking* mpTracker;
    Atlas* mpAtlas;

    bool mBoth;
    double mTimeStamp = 0.0f;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    std::vector<std::pair<KeyFrame*, KeyFrame*>> mLoopKeyFrames;
    std::mutex mMutexLoop;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;
    
    std::mutex mMutexCamera;
    Sophus::SE3f mCameraPose;
    Sophus::SE3f mTbc;

    ros::Publisher mCamPosePub;
    ros::Publisher mCamPathPub;
    ros::Publisher mAllPointCloudPub;
    ros::Publisher mRefPointCloudPub;
    ros::Publisher mKeyFramePub;
    
    image_transport::Publisher mDrawFramePub;
    tf::TransformBroadcaster mBroadcaster;
    ros::NodeHandle nh;

    void PubCameraPoseAndTF(std_msgs::Header& header);
    void PubCameraPath(std_msgs::Header& header);
    void PubPointCloud(std_msgs::Header& header);
    void PubFrame();
    void PubKeyFrame(std_msgs::Header& header);

    void publish_ros_tf_transform(const Sophus::SE3f& Twc_SE3f, const string& frame_id, const string& child_frame_id, ros::Time msg_time);
    static tf::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f);
};

}


#endif // ROS_VIEWER_H


