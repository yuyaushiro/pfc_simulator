#pragma once

#include <cmath>

#include "cmd_vel.h"
#include "mcl.h"
#include "pfc.h"
#include "pose.h"


/// ロボット
class Robot
{
public:
  /// コンストラクタ
  Robot();
  Robot(const Pose& initialPose, const Mcl& mcl, const Pfc& pfc);

  ~Robot() = default;

  /// 状態を遷移させる
  static Pose transitionState(const CmdVel& cmdVel, double dt, const Pose& pose);

  /// 1ステップ進める
  void oneStep(double dt);

  /// 描画する
  void draw();

private:
  /// 姿勢
  Pose pose_;

  /// 推定器
  Mcl mcl_;

  /// エージェント
  Pfc pfc_;

  /// 軌跡
  std::vector<Pose> trajectory_;

  /// 描画半径
  double radius_;
};
