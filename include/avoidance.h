#pragma once

#include <iostream>
#include <vector>


/// 回避
class Avoidance
{
public:
  /// コンストラクタ
  Avoidance();

  /// コンストラクタ
  Avoidance(double weight, double maxWeight, double decreaseTime);

  /// 重みの取得
  double getWeight();

  /// 重み候補の追加
  void addWeightCandidate(double reward);

  /// 重みの更新
  void updateWeight(int candidateIndex);

  /// 重みの減少
  void decreaseWeight(double dt);

private:
  /// 重み
  double weight_;

  /// 最大重み
  double maxWeight_;

  /// 最小重み
  double minWeight_;

  /// 減少時間
  double decreaseTime_;

  /// 重み候補
  std::vector<double> weightCandidates_;
};
