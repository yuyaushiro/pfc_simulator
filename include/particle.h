#pragma once

#include <cmath>
#include <vector>
#include <random>

#include "cmd_vel.h"
#include "pose.h"
#include "goal.h"
#include "avoidance.h"


/// パーティクル
class Particle
{
public:
  /// コンストラクタ
  Particle();
  Particle(const Pose& initialPose, double weight);

  /// デストラクタ
  ~Particle() = default;

  /// 動作による更新
  void updateWithMotion(const CmdVel& cmdVel, double dt, const std::vector<double>& ns);

  /// ゴール観測による更新
  void updateWithGoalObservation(const Goal& goal);

  /// 重みの取得
  double getWeight();

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
