//
// Created by robot on 23-12-2.
//
#include <CeresOptimizer.h>
namespace ORB_SLAM3
{
bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
{
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> se3(x);
  Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(delta);
  Eigen::Map<Eigen::Matrix<double, 6, 1>> plus(x_plus_delta);
  plus = (Sophus::SE3d::exp(update) * Sophus::SE3d::exp(se3)).log();
}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
{
  Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> leftJacobian(jacobian);
  leftJacobian.setIdentity();
}

PoseOnly::PoseOnly(Eigen::Vector3d &p3d, Eigen::Vector2d &pixel, double information,
                   ORB_SLAM3::GeometricCamera *pCamera) : mXw(p3d), mObs(pixel), mpCamera(pCamera)
{
  mInformation.setIdentity();
  mInformation *= information;
}

bool PoseOnly::Evaluate(const double *const *parameter, double *residuals, double **jacobians) const
{
  double pCam[3];
  se3TransPoint(parameter[0], mXw.data(), pCam);
  Eigen::Vector3d Xc;
  Xc << pCam[0], pCam[1], pCam[2];

  Eigen::Map<Eigen::Vector2d> err(residuals);
  err = mInformation * mObs - mpCamera->project(Xc);

  if (!jacobians)
    return true;

  double *jacobian_uv = jacobians[0];

  if (!jacobian_uv)
    return true;

  Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J(jacobian_uv);
  double x = pCam[0];
  double y = pCam[1];
  double z = pCam[2];

  Eigen::Matrix<double, 3, 6> SE3deriv;
  SE3deriv << 0.f, z, -y, 1.f, 0.f, 0.f,
      -z, 0.f, x, 0.f, 1.f, 0.f,
      y, -x, 0.f, 0.f, 0.f, 1.f;

  J = -mpCamera->projectJac(Xc) * SE3deriv;
  return true;
}

int CeresOptimizer::PoseOptimization(Frame *pFrame)
{
  ceres::Problem problem;
  const int N = pFrame->N;
  const float deltaMono = std::sqrt(5.991f);
  const float deltaStereo = std::sqrt(7.815f);

  int nInitialCorrespondences = 0;

  auto* pose_ptr = new double[3];
  Eigen::Map<Eigen::Matrix<double, 6, 1>> pose(pose_ptr);
  pose = pFrame->GetPose().log().cast<double>();

//  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose*> vpEdgesMono;
//  vector<ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody*> vpEdgesMono_FHR;
//  vector<size_t> vnIndexEdgeMono, vnIndexEdgeRight;
//  vpEdgesMono.reserve(N);
//  vpEdgesMono_FHR.reserve(N);
//  vnIndexEdgeMono.reserve(N);
//  vnIndexEdgeRight.reserve(N);
//
//  vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose*> vpEdgesStereo;
//  vector<size_t> vnIndexEdgeStereo;
//  vpEdgesStereo.reserve(N);
//  vnIndexEdgeStereo.reserve(N);

  {
    unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    for (int i = 0; i < N; i++)
    {
      MapPoint *pMP = pFrame->mvpMapPoints[i];
      if (pMP)
      {
        //Conventional SLAM
        if(!pFrame->mpCamera2)
        {
          // Monocular observation
          if (pFrame->mvuRight[i] < 0)
          {
            nInitialCorrespondences++;
            pFrame->mvbOutlier[i] = false;

            Eigen::Matrix<double, 2, 1> obs;
            const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
            obs << kpUn.pt.x, kpUn.pt.y;

            Eigen::Vector3d Xw = pMP->GetWorldPos().cast<double>();
            const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
            ceres::CostFunction* costFunction = new PoseOnly(Xw, obs, invSigma2, pFrame->mpCamera);
            ceres::LossFunction* loss_function = new ceres::HuberLoss(deltaMono);
            problem.AddResidualBlock(costFunction, loss_function, pose_ptr);
          }
        }
      }
    }
  }
  return 0;
}

void CeresOptimizer::SetFramePoses(const std::vector<Frame> &vFrame)
{
  frameIdToParamId.reserve(vFrame.size());
  framePoses.resize(Eigen::NoChange, vFrame.size());
  for (int i = 0; i < vFrame.size(); i++)
  {
    framePoses.col(i) = vFrame[i].GetPose().log().cast<double>();
    frameIdToParamId.insert({vFrame[i].mnId, i});
  }
}

double *CeresOptimizer::GetFramePose(int frameId)
{
  if (frameIdToParamId.count(frameId) > 0)
  {
    const int paramId = frameIdToParamId[frameId];
    return framePoses.col(paramId).data();
  }
  else
  {
    return nullptr;
  }
}
}
