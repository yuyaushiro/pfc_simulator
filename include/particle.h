#pragma once

#include <cmath>
#include <vector>
#include <random>

#include "cmd_vel.h"
#include "pose.h"
#include "avoidance.h"


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

  /// 回避重みの取得
  double getAvoidanceWeight();

  /// 回避重みの候補を追加
  void addAvoidanceWeightCandidate(double reward);

  /// 回避重みの更新
  void updateAvoidanceWeight(int candidateIndex);

  /// 重みの減少
  void decreaseAvoidanceWeight(double dt);

  /// 描画
  void draw();

  /// 姿勢
  Pose pose_;

private:
  /// 重み
  double weight_;

  /// 半径
  double radius_;

  /// 回避
  Avoidance avoidance_;
};
