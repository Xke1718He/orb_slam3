#include "RosViewer.h"

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry> 


#include <opencv2/core/eigen.hpp>

#include <mutex>

namespace ORB_SLAM3
{

ROSViewer::ROSViewer(System* pSystem, Tracking* pTracker, Atlas* pAtlas, FrameDrawer *pFrameDrawer):
   mpSystem(pSystem), mpFrameDrawer(pFrameDrawer), mpTracker(pTracker), mpAtlas(pAtlas),
   mBoth(false), mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    const std::string prefix = "/ORB_SLAM3";
    mCamPosePub = nh.advertise<geometry_msgs::PoseStamped >(prefix + "/camera_pose",1);
    mCamPathPub = nh.advertise<nav_msgs::Path>(prefix + "/camera_path",1);
    mAllPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/point_cloud_all",1);
    mRefPointCloudPub = nh.advertise<sensor_msgs::PointCloud2>(prefix + "/point_cloud_ref",1);
    mKeyFramePub = nh.advertise<visualization_msgs::MarkerArray>(prefix + "/keyframes", 1);

    
    image_transport::ImageTransport it_(nh);
    mDrawFramePub = it_.advertise(prefix + "/frame", 1);

    mTbc = mpTracker->GetTbc();
}

void ROSViewer::PubFrame()
{
    cv::Mat toShow;
    cv::Mat im = mpFrameDrawer->DrawFrame(1.f);

    if(mBoth){
       cv::Mat imRight = mpFrameDrawer->DrawRightFrame(1.0f);
       cv::hconcat(im,imRight,toShow);
    }
    else{
       toShow = im;
    }
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time::now();
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", toShow).toImageMsg();
    mDrawFramePub.publish(imgTrackMsg);
}

void ROSViewer::Run()
{
    mbFinished = false;
    mbStopped = false;
    while(true)
    {
      std_msgs::Header header;
      header.frame_id = "world";
      double time;
      {
          std::unique_lock<std::mutex> lock(mMutexCamera);
          time = mTimeStamp;
      }
      header.stamp = ros::Time(time);

      PubFrame();
      PubCameraPoseAndTF(header);
      PubCameraPath(header);
      PubPointCloud(header);

      if(CheckFinish())
          break;
      usleep(1000);
    }

    SetFinish();
}

void ROSViewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ROSViewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ROSViewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool ROSViewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void ROSViewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool ROSViewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ROSViewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
      mbStopped = true;
      mbStopRequested = false;
      return true;
    }

    return false;
}

void ROSViewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void ROSViewer::SetCurrentCameraPoseAndTime(const Sophus::SE3f &Tcw, double time)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw;
    mTimeStamp = time;
}

void ROSViewer::PubCameraPoseAndTF(std_msgs::Header& header)
{
    Sophus::SE3f Twc;
    Sophus::SE3f Twb;
    {
        unique_lock<mutex> lock(mMutexCamera);
        Twb = mCameraPose.inverse();
        Twc = Twb * mTbc;
    }
    geometry_msgs::PoseStamped camPose;
    camPose.header = header;

    camPose.pose.position.x = Twc.translation().x();
    camPose.pose.position.y = Twc.translation().y();
    camPose.pose.position.z = Twc.translation().z();

    camPose.pose.orientation.w = Twc.unit_quaternion().coeffs().w();
    camPose.pose.orientation.x = Twc.unit_quaternion().coeffs().x();
    camPose.pose.orientation.y = Twc.unit_quaternion().coeffs().y();
    camPose.pose.orientation.z = Twc.unit_quaternion().coeffs().z();
    mCamPosePub.publish(camPose);

    publish_ros_tf_transform(Twb, "world", "body", header.stamp);
    publish_ros_tf_transform(mTbc, "body", "camera", header.stamp);
}

void ROSViewer::PubCameraPath(std_msgs::Header& header)
{
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker keyPoses;
    visualization_msgs::Marker markerEdge;
    keyPoses.header = header;
    keyPoses.ns = "keyPoses";
    keyPoses.type = visualization_msgs::Marker::POINTS;
    keyPoses.action = visualization_msgs::Marker::ADD;
    keyPoses.pose.orientation.w = 1.0;
    keyPoses.lifetime = ros::Duration();

    keyPoses.id = 0;
    keyPoses.scale.x = 0.05;
    keyPoses.scale.y = 0.05;
    keyPoses.scale.z = 0.05;
    keyPoses.color.b = 1.0;
    keyPoses.color.a = 1.0;

    nav_msgs::Path camPath;
    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();

    if (vpKFs.empty())
    {
        return;
    }
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    for (auto pKF : vpKFs)
    {
        const auto pose = pKF->GetPoseInverse();
        const auto time = pKF->mTimeStamp;
        geometry_msgs::PoseStamped camPose;
        camPose.header.frame_id = "world";
        camPose.header.stamp = ros::Time(time);

        camPose.pose.position.x = pose.translation().x();
        camPose.pose.position.y = pose.translation().y();
        camPose.pose.position.z = pose.translation().z();

        camPose.pose.orientation.w = pose.unit_quaternion().coeffs().w();
        camPose.pose.orientation.x = pose.unit_quaternion().coeffs().x();
        camPose.pose.orientation.y = pose.unit_quaternion().coeffs().y();
        camPose.pose.orientation.z = pose.unit_quaternion().coeffs().z();
        camPath.poses.push_back(camPose);

        geometry_msgs::Point poseMarker;
        poseMarker.x = pose.translation().x();
        poseMarker.y = pose.translation().y();
        poseMarker.z = pose.translation().z();
        keyPoses.points.push_back(poseMarker);
    }

    markerEdge.header = header;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.05;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    std::vector<std::pair<KeyFrame*, KeyFrame*>> loopKeyFrames;
    {
        std::unique_lock<std::mutex> lock(mMutexLoop);
        loopKeyFrames.insert(loopKeyFrames.end(), mLoopKeyFrames.begin(), mLoopKeyFrames.end());
    }

    //show loop info
    for (auto& loopKeyFrame : loopKeyFrames)
    {
       Sophus::SE3f curPose = loopKeyFrame.first->GetPoseInverse();
       Sophus::SE3f matchPose = loopKeyFrame.second->GetPoseInverse();

       geometry_msgs::Point p;
       p.x = curPose.translation().x();
       p.y = curPose.translation().y();
       p.z = curPose.translation().z();
       markerEdge.points.push_back(p);
       p.x = matchPose.translation().x();
       p.y = matchPose.translation().y();
       p.z = matchPose.translation().z();
       markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerEdge);
    markerArray.markers.push_back(keyPoses);

    if (camPath.poses.empty())
    {
        return;
    }
    camPath.header = header;
    mCamPathPub.publish(camPath);
    mKeyFramePub.publish(markerArray);
}

void ROSViewer::PubPointCloud(std_msgs::Header& header)
{
    sensor_msgs::PointCloud2 all_point_cloud, ref_point_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr allPoints( new pcl::PointCloud<pcl::PointXYZRGBA> );
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr refPoints( new pcl::PointCloud<pcl::PointXYZRGBA> );

    const vector<MapPoint*> &vpMPs = mpAtlas->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpAtlas->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty() && vpRefMPs.empty())
        return;

    for (auto vpMP : vpMPs)
    {
      if(vpMP->isBad() || spRefMPs.count(vpMP))
          continue;
      Eigen::Vector3f pos = vpMP->GetWorldPos();
      pcl::PointXYZRGBA p1;
      p1.x = pos(0);
      p1.y = pos(1);
      p1.z = pos(2);
      p1.b = 255;
      p1.g = 255;
      p1.r = 255;
      p1.a = 255;
      allPoints->points.push_back(p1);
    }
    pcl::PCLPointCloud2 pointCloud2;
    pcl::toPCLPointCloud2(*allPoints, pointCloud2);    // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pointCloud2, all_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    all_point_cloud.header = header;

    for (auto spRefMP : spRefMPs)
    {
      if(spRefMP->isBad())
          continue;
      auto pos = spRefMP->GetWorldPos();
      pcl::PointXYZRGBA p2;
      p2.x = pos(0);
      p2.y = pos(1);
      p2.z = pos(2);
      p2.b = 0;
      p2.g = 0;
      p2.r = 255;
      p2.a = 255;
      refPoints->points.push_back( p2 );
    }
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*refPoints, pcl_pc2); // pcl::PointXYZRGBA -> pcl::PCLPointCloud2
    pcl_conversions::fromPCL(pcl_pc2, ref_point_cloud);  // pcl::PCLPointCloud2 -> sensor_msgs::PointCloud2
    ref_point_cloud.header = header;
    mAllPointCloudPub.publish(all_point_cloud);
    mRefPointCloudPub.publish(ref_point_cloud);
}

void ROSViewer::publish_ros_tf_transform(const Sophus::SE3f& Twc, const string& frame_id, const string& child_frame_id, ros::Time msg_time)
{
    tf::Transform tf_transform = SE3f_to_tfTransform(Twc);
    mBroadcaster.sendTransform(tf::StampedTransform(tf_transform, msg_time, frame_id, child_frame_id));
}

tf::Transform ROSViewer::SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf::Transform(R_tf, t_tf);
}
void ROSViewer::SetLoopFrame(KeyFrame *pCurrentFrame, KeyFrame *pLoopMatchedKF)
{
    std::unique_lock<std::mutex> lock(mMutexLoop);
    mLoopKeyFrames.emplace_back(pCurrentFrame, pLoopMatchedKF);
}
}


