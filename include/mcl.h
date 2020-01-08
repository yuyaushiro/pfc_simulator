#pragma once

#include <vector>
#include <GLFW/glfw3.h>

#include "particle.h"
#include "pose.h"
#include "goal.h"
#include "cmd_vel.h"


// モンテカルロローカライゼーション
class Mcl
{
public:
  /// コンストラクタ
  Mcl();
  Mcl(const Pose& initialPose, int particleNum);

  /// デストラクタ
  ~Mcl() = default;

  /// モーションアップデート
  void updateWithMotion(const CmdVel& cmdVel, double dt);

  /// ゴール観測によるアップデート
  void updateWithGoalObservation(const Goal& goal);

  /// リサンプリング
  void resampling();

  /// 描画
  void draw();

  /// パーティクル
  std::vector<Particle> particles_;

private:
  /// パーティクル数
  int particleNum_;
};
