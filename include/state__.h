#pragma once

#include <string>
#include <vector>

#include "pose.h"
#include "cmd_vel.h"


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
  State(const std::string& fileName, const std::vector<int>& cellNum,
        const std::vector<double>& cellWidth, const Pose& minPose,
        const std::vector<CmdVel>& cmdVels);

private:
  /// セルインデックスに変換
  std::vector<int> toCellIndex(const Pose& pose) const;

  /// 状態インデックスに変換
  unsigned long State::toStateIndex(std::vector<int>& cellIndex) const;

  /// 状態価値ファイルを読み込む
  void loadValueFile();

  /// 状態遷移ファイルを読み込む
  void loadTransFile();

  /// 1行を分割
  std::vector<std::string> splitOneLine(std::string& line, char delimiter);

  /// 行動のインデックスを取得
  int State::getActionIndex(const std::string& actionName) const;

private:
  /// 価値関数ファイル名
  std::string valueFileName_;

  /// 状態遷移ファイル名
  std::string transFileName_;

  /// 価値
  std::vector<unsigned long> value_;

  /// 遷移確率 [action][theta_i]
  std::vector< std::vector< std::vector<Transition> > > transProb_;

  /// 行動
  std::vector<CmdVel> cmdVels_;

  /// 状態数
  std::vector<int> cellNum_;

  /// 1セルあたりの幅 [m/cell, m/cell, rad/cell]
  std::vector<double> cellWidth_;

  /// 最小の姿勢
  Pose minPose_;
};
