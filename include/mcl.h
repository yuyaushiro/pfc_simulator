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
  Mcl(const Pose& initialPose, int particleNum, int seedValue,
      const std::vector<double> initPoseStd, const std::vector<double> motionStd);

  /// デストラクタ
  ~Mcl() = default;

  /// 初期化
  void init(const Pose& initialPose, const std::vector<double> initPoseStd);

  /// モーションアップデート
  void updateWithMotion(const CmdVel& cmdVel, double dt);

  /// ゴール観測によるアップデート
  void updateWithGoalObservation(const Goal& goal);

  /// リサンプリング
  void resampling();

  /// 平均姿勢を計算
  void calcAveragePose();

  /// 描画
  void draw();

  /// パーティクル
  std::vector<Particle> particles_;

  /// 平均姿勢
  Pose averagePose_;

private:
  /// パーティクル数
  int particleNum_;

  /// メルセンヌツイスター
  std::mt19937 mt_;

  /// 動作ノイズ
  std::vector<double> motionStd_;
};
