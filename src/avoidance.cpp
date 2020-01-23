#include "avoidance.h"


// コンストラクタ
//------------------------------------------------------------------------------
Avoidance::Avoidance()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Avoidance::Avoidance(double weight, double maxWeight, double decreaseTime)
  : weight_(weight)
  , minWeight_(1.0)
  , maxWeight_(maxWeight)
  , decreaseTime_(decreaseTime)
{}


// 重みの取得
//------------------------------------------------------------------------------
double Avoidance::getWeight()
{
  double weight;
  // 現在の重みと最新の候補から大きい方を重みとして返す
  (weightCandidates_.size() > 0) ?
    weight = std::max(weight_, weightCandidates_.back()) :
    weight = weight_;
  return weight;
}


// 重み候補の追加
//------------------------------------------------------------------------------
void Avoidance::addWeightCandidate(double reward)
{
  // 報酬が通常の遷移より小さい？
  (reward < -0.5) ?
    weightCandidates_.push_back(maxWeight_) :
    weightCandidates_.push_back(minWeight_);
}


// 重みの更新
//------------------------------------------------------------------------------
void Avoidance::updateWeight(int candidateIndex)
{
  if (weightCandidates_.size() != 0)
  {
    std::vector<double>::iterator maxIterator =
      std::max_element(weightCandidates_.begin(), weightCandidates_.end());
    weight_ = std::max(weight_, *maxIterator);
    weightCandidates_.resize(0);
  }
  // weight_ = std::max(weight_, weightCandidates_[candidateIndex]);
  // weightCandidates_.resize(0);
}


// 重みの減少
//------------------------------------------------------------------------------
void Avoidance::decreaseWeight(double dt)
{
  // 減少値
  double decreaseValue = (maxWeight_ - minWeight_) * dt / decreaseTime_;
  if (weight_ > minWeight_)
    weight_ -= decreaseValue;
  if (weight_ < minWeight_)
    weight_ = minWeight_;
}
