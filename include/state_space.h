#pragma once

#include <string>
#include <vector>

#include "cmd_vel.h"
#include "pose.h"
#include "state.h"


/// 状態空間
class StateSpace
{
public:
  /// コンストラクタ
  StateSpace();

  /// コンストラクタ
  StateSpace(const std::string& fileName, const std::vector<int>& cellNum,
             const std::vector<double>& cellWidth, const Pose& minPose,
             const std::vector<CmdVel>& cmdVels_);

  /// 状態価値を取得
  double getValue(const Pose& pose) const;

  /// 行動価値を取得
  double getActionValue(const Pose& pose, const CmdVel& cmdVel) const;

  /// 状態遷移の報酬を取得
  double getReward(const Pose& pose, const CmdVel& cmdVel) const;

  /// 状態遷移後の価値を取得
  double getPosteriorValue(const Pose& pose, const CmdVel& cmdVel) const;

private:
  /// 姿勢を状態へ変換
  unsigned long toStateIndex(const Pose& pose) const;

  /// 状態価値ファイルを読み込む
  void loadValueFile();

  /// 状態遷移ファイルを読み込む
  void loadTransFile();

  /// 状態遷移分割
  bool parseStateTrans(std::string& line);

  /// 1行を分割
  std::vector<std::string> splitOneLine(std::string& line, char delimiter);

  /// 行動のインデックスを取得
  int getActionIndex(const std::string& actionName) const;

private:
  /// 価値関数ファイル名
  std::string valueFileName_;

  /// 状態遷移ファイル名
  std::string transFileName_;

  /// 状態
  std::vector<State> state_;

  /// 行動
  std::vector<CmdVel> cmdVels_;

  /// 状態数
  std::vector<int> cellNum_;

  /// 1セルあたりの幅 [m/cell, m/cell, rad/cell]
  std::vector<double> cellWidth_;

  /// 最小の姿勢
  Pose minPose_;
};
