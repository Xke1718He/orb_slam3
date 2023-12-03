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
  mInformation.diagonal() << information, information;
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

int static CeresOptimizer::PoseOptimization(Frame *pFrame)
{

  return 0;
}
}
