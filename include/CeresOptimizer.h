//
// Created by robot on 23-12-2.
//

#ifndef ORB_SLAM3_WS_CERESOPTIMIZER_H
#define ORB_SLAM3_WS_CERESOPTIMIZER_H
#include <ceres/ceres.h>
#include <glog/logging.h>
#include <sophus/se3.hpp>
#include <utility>
#include <include/CameraModels/GeometricCamera.h>
#include "Frame.h"
#include "MapPoint.h"
namespace ORB_SLAM3
{
class PoseLocalParameterization : public  ceres::LocalParameterization
{
  virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
  virtual bool ComputeJacobian(const double *x, double *jacobian) const;
  virtual int GlobalSize() const { return 6; };
  virtual int LocalSize() const { return 6; };
};

class PoseOnly : public ceres::SizedCostFunction<2, 6>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PoseOnly(Eigen::Vector3d &p3d, Eigen::Vector2d &pixel, double information, ORB_SLAM3::GeometricCamera* pCamera);
  bool Evaluate(double const* const* parameter, double* residuals, double** jacobians) const override;
private:
  Eigen::Vector3d mXw;
  Eigen::Vector2d mObs;
  ORB_SLAM3::GeometricCamera* mpCamera;
  Eigen::Matrix2d mInformation;
};

class CeresOptimizer{
public:
  struct ResidualBlock{
      ResidualBlock(std::shared_ptr<ceres::CostFunction> _cost_function,
                    std::shared_ptr<ceres::LossFunction> _loss_function,
                    const std::vector<double*>& _parameter_blocks)
                    : cost_function(std::move(_cost_function)), loss_function(std::move(_loss_function)),
                    parameter_blocks(_parameter_blocks) {}

      std::shared_ptr<ceres::CostFunction> cost_function;
      std::shared_ptr<ceres::LossFunction> loss_function;
      std::vector<double*> parameter_blocks;
  };

  int PoseOptimization(Frame* pFrame);
  void SetFramePoses(const std::vector<Frame>& vFrame);
  double* GetFramePose(int frameId);
private:
  std::unordered_map<int, int> frameIdToParamId;
  Eigen::Matrix<double, 6, Eigen::Dynamic> framePoses;
};

template<typename T>
void se3TransPoint(const T se3[6], const T pt[3], T result[3])
{
  const T upsilon[3] = {se3[3], se3[4], se3[5]};

  const T& a0 = se3[0];
  const T& a1 = se3[1];
  const T& a2 = se3[2];
  const T theta2 = a0 * a0 + a1 * a1 + a2 * a2;

  if (theta2 > T(std::numeric_limits<double>::epsilon()))
  {
    const T theta = sqrt(theta2);
    const T costheta = cos(theta);
    const T sintheta = sin(theta);
    const T theta_inverse = T(1.0) / theta;

    const T w[3] = { a0 * theta_inverse,
                     a1 * theta_inverse,
                     a2 * theta_inverse };

    const T w_cross_pt[3] = { w[1] * pt[2] - w[2] * pt[1],
                              w[2] * pt[0] - w[0] * pt[2],
                              w[0] * pt[1] - w[1] * pt[0] };
    const T tmp =
        (w[0] * pt[0] + w[1] * pt[1] + w[2] * pt[2]) * (T(1.0) - costheta);

    result[0] = pt[0] * costheta + w_cross_pt[0] * sintheta + w[0] * tmp;
    result[1] = pt[1] * costheta + w_cross_pt[1] * sintheta + w[1] * tmp;
    result[2] = pt[2] * costheta + w_cross_pt[2] * sintheta + w[2] * tmp;

    const T w_cross_upsilon[3] = { w[1] * upsilon[2] - w[2] * upsilon[1],
                                   w[2] * upsilon[0] - w[0] * upsilon[2],
                                   w[0] * upsilon[1] - w[1] * upsilon[0] };

    const T w_double_cross_upsilon[3] = { w[1] * w_cross_upsilon[2] - w[2] * w_cross_upsilon[1],
                                          w[2] * w_cross_upsilon[0] - w[0] * w_cross_upsilon[2],
                                          w[0] * w_cross_upsilon[1] - w[1] * w_cross_upsilon[0] };

    result[0] += upsilon[0] + ((T(1.0)-costheta)/theta) * w_cross_upsilon[0]
                 + ((theta-sintheta)/theta) * w_double_cross_upsilon[0];
    result[1] += upsilon[1] + ((T(1.0)-costheta)/theta) * w_cross_upsilon[1]
                 + ((theta-sintheta)/theta) * w_double_cross_upsilon[1];
    result[2] += upsilon[2] + ((T(1.0)-costheta)/theta) * w_cross_upsilon[2]
                 + ((theta-sintheta)/theta) * w_double_cross_upsilon[2];


  }
  else
  {
    const T w_cross_pt[3] = { a1 * pt[2] - a2 * pt[1],
                              a2 * pt[0] - a0 * pt[2],
                              a0 * pt[1] - a1 * pt[0] };

    result[0] = pt[0] + w_cross_pt[0];
    result[1] = pt[1] + w_cross_pt[1];
    result[2] = pt[2] + w_cross_pt[2];

    const T w_cross_upsilon[3] = { a1 * upsilon[2] - a2 * upsilon[1],
                                   a2 * upsilon[0] - a0 * upsilon[2],
                                   a0 * upsilon[1] - a1 * upsilon[0] };

    result[0] += upsilon[0] + w_cross_upsilon[0];
    result[1] += upsilon[1] + w_cross_upsilon[1];
    result[2] += upsilon[2] + w_cross_upsilon[2];
  }
}
}
#endif //ORB_SLAM3_WS_CERESOPTIMIZER_H
