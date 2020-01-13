#pragma once

#include <cmath>

#include "cmd_vel.h"
#include "goal.h"
#include "mcl.h"
#include "pose.h"
#include "pfc.h"


/// ロボット
class Robot
{
public:
  /// コンストラクタ
  Robot();

  /// コンストラクタ
  Robot(const Pose& initialPose, const Goal& goal, const Mcl& mcl, const Pfc& pfc,
        int seedValue, const std::vector<double> motionStd);

  /// デストラクタ
  ~Robot() = default;

  /// 状態を遷移させる（ノイズあり）
  Pose transitionStateWithNoise(const CmdVel& cmdVel, double dt, const Pose& pose);

  /// 状態を遷移させる
  static Pose transitionState(const CmdVel& cmdVel, double dt, const Pose& pose);

  /// 1ステップ進める
  void oneStep(double dt);

  /// 姿勢の取得
  Pose getPose();

  /// リスタート
  void restart(const Pose& initialPose, const std::vector<double> initPoseStd);

  /// 描画する
  void draw();

private:
  /// 姿勢
  Pose pose_;

  /// ゴール
  Goal goal_;

  /// 推定器
  Mcl mcl_;

  /// エージェント
  Pfc pfc_;

  /// メルセンヌツイスター
  std::mt19937 mt_;

  /// 動作ノイズ
  std::vector<double> motionStd_;

  /// 軌跡
  std::vector<Pose> trajectory_;

  /// 描画半径
  double radius_;
};
