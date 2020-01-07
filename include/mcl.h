#pragma once

#include <vector>
#include <GLFW/glfw3.h>

#include "particle.h"
#include "pose.h"
#include "cmd_vel.h"


// モンテカルロローカライゼーション
class Mcl
{
public:
  /// コンストラクタ
  Mcl();
  Mcl(Pose& initialPose, int particleNum);

  /// デストラクタ
  ~Mcl() = default;

  /// モーションアップデート
  void updateWithMotion(CmdVel& cmdVel, double dt);

  /// 描画
  void draw();

  /// パーティクル
  std::vector<Particle> particles_;

private:
  /// パーティクル数
  int particleNum_;
};
