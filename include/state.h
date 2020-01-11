#pragma once

#include <string>
#include <vector>

#include "pose.h"
#include "cmd_vel.h"
#include "grid_map.h"


/// 状態遷移
typedef struct Transition_
{
  /// 確率
  double probability = 0;

  /// 後の状態
  std::vector<int> deltaTransCell;
} Transition;


/// 状態
class State
{
public:
  /// コンストラクタ
  State();

  /// コンストラクタ
  State(const std::string& fileName, const GridMap& gridMap, const std::vector<CmdVel>& cmdVels,
        const std::vector<int>& cellNum, const std::vector<double>& cellWidth, const Pose& pose);

  /// 価値を取得
  double getValue(const Pose& pose) const;

  /// 遷移後の価値
  double getPosteriorValue(const Pose& pose, const CmdVel& cmdVel) const;

  /// 行動による報酬を取得
  double getReward(const Pose& pose, const CmdVel& cmdVel) const;

  /// セルインデックスへ変換
  std::vector<int> toCellIndex(const Pose& pose) const;

  /// 状態インデックスへ変換
  unsigned long toStateIndex(const Pose& pose) const;

  /// 価値関数ファイルを読み込む
  void loadValueFile();

  /// 状態遷移確率ファイルを読み込む
  void loadTransProbFile();

  /// 1行を分割
  std::vector<std::string> splitOneLine(std::string& line, char delimiter) const;

  /// 行動のインデックスを取得
  int getActionIndex(const std::string& actionName) const;

private:
  /// 価値関数ファイル名
  std::string valueFileName_;

  /// 遷移確率ファイル名
  std::string transProbFileName_;

  /// 価値関数
  std::vector<size_t> values_;

  /// 遷移確率 [action][theta_i]
  std::vector< std::vector< std::vector<Transition> > > transProb_;

  /// 行動
  std::vector<CmdVel> cmdVels_;

  /// 格子地図
  GridMap gridMap_;

  /// 状態数
  std::vector<int> cellNum_;

  /// 1セルあたりの幅 [m/cell, m/cell, rad/cell]
  std::vector<double> cellWidth_;

  Pose minPose_;
};
