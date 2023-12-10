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
  err = mInformation * ( mObs - mpCamera->project(Xc));

  mchi2 = err.dot(mInformation * err);

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

void CeresOptimizer::AddResidualBlock(const ResidualBlock& residualInfo)
{
  vresidualInfo.push_back(residualInfo);
}

int CeresOptimizer::PoseOptimization(Frame *pFrame)
{
  const int N = pFrame->N;
  const float deltaMono = std::sqrt(5.991f);
  const float deltaStereo = std::sqrt(7.815f);

  int nInitialCorrespondences = 0;

  std::vector<Frame*> vFrame;
  vFrame.push_back(pFrame);

  SetFramePoses(vFrame);
  double* pose = GetFramePose(pFrame->mnId);

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
            ceres::LossFunction* lossFunction = new ceres::HuberLoss(deltaMono);
            ResidualBlock residualInfo(costFunction, lossFunction, {pose});
            AddResidualBlock(residualInfo);
          }
        }
      }
    }
  }

  if(nInitialCorrespondences<3)
      return 0;

  // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
  // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
  const float chi2Mono[4]={5.991,5.991,5.991,5.991};
  const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
  const int its[4]={10,10,10,10};

  int nBad=0;
  for(size_t it=0; it<4; it++)
  {
    ceres::Problem problem;
    ceres::LocalParameterization* poseLocalParameter = new PoseLocalParameterization();
    std::vector<ResidualBlock> vResidualInfo = GetAllResidualBlock();
    for (int i=0; i < vResidualInfo.size; i++)
    {
      if (!vresidualInfo[i].active)
      {
        continue;
      }
      auto parameterBlocks = vresidualInfo[i].parameter_blocks;
      auto costFunction = vResidualInfo[i].cost_function;
      auto& lossFunction = vresidualInfo[i].loss_function;

      problem.AddResidualBlock(costFunction.get(), lossFunction.get(), parameterBlocks);
    }

    problem.SetParameterization(pose, poseLocalParameter);
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    for (int i=0; i < vResidualInfo.size; i++)
    {
      if (!vresidualInfo[i].active)
      {
        continue;
      }
      auto costFunction = vResidualInfo[i].cost_function;
      PoseOnly* costFunctionPose = std::static_cast<PoseOnly>(costFunction.get());
      if (costFunctionPose->chi2() > chi2Mono[it])
      {
        // pFrame->mvbOutlier[idx]=true;
        vresidualInfo[i].active = false;
        nBad++;
      }
    }
  }
  return 0;
}

void CeresOptimizer::SetFramePoses(const std::vector<Frame*> &vpFrame)
{
  frameIdToParamId.reserve(vFrame.size());
  framePoses.resize(Eigen::NoChange, vFrame.size());
  for (int i = 0; i < vFrame.size(); i++)
  {
    framePoses.col(i) = vFrame[i]->GetPose().log().cast<double>();
    frameIdToParamId.insert({vFrame[i]->mnId, i});
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
