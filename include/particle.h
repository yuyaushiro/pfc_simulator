#pragma once

#include <cmath>
#include <vector>
#include <random>

#include "cmd_vel.h"
#include "pose.h"


/// パーティクル
class Particle
{
public:
  /// コンストラクタ
  Particle();
  Particle(Pose& initialPose, double weight);

  /// デストラクタ
  ~Particle() = default;

  /// 動作による更新
  void transitionStateWithNoise(CmdVel& cmdVel, double dt, const std::vector<double>& ns);

  /// 描画
  void draw();

  /// 姿勢
  Pose pose_;

private:
  /// 重み
  double weight_;

  /// 半径
  double radius_;
};
