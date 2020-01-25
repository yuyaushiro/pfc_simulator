#include "pfc.h"
#include "robot.h"


// コンストラクタ
//------------------------------------------------------------------------------
Pfc::Pfc()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Pfc::Pfc(const std::vector<CmdVel>& cmdVels, const State& state, double magnitude)
  : cmdVels_(cmdVels)
  , state_(state)
  , magnitude_(magnitude)
  , history_{cmdVels[0]}
{}


// 意思決定
//------------------------------------------------------------------------------
CmdVel Pfc::decisionMaking(std::vector<Particle>& particles, double dt)
{
  // std::cout << "---" << std::ndl;
  // 行動価値のリスト
  std::vector<double> actionValues(cmdVels_.size());
  for (int i = 0; i < cmdVels_.size(); i++)
  {
    actionValues[i] = evaluateAction(cmdVels_[i], particles);
    // std::cout << actionValues[i] << std::endl;
  }

  // argmax
  std::vector<double>::iterator maxIterator = std::max_element(actionValues.begin(), actionValues.end());
  int maxIndex = std::distance(actionValues.begin(), maxIterator);
  // std::cout << maxIndex << std::endl;
  // std::cout << std::endl;

  // 回避重みを更新する
  for (Particle& particle : particles)
  {
    particle.decreaseAvoidanceWeight(dt);
    particle.updateAvoidanceWeight(maxIndex);
  }

  CmdVel cmdVel = cmdVels_[maxIndex];
  // 行動がループ
  if (cmdVels_[maxIndex].nu + history_.back().nu == 0 && cmdVels_[maxIndex].omega + history_.back().omega == 0)
    cmdVel = cmdVels_[0];

  // 行動の履歴を追加
  history_.push_back(cmdVel);

  return cmdVel;
}


// 行動の評価（Q-PFCの計算）
//------------------------------------------------------------------------------
double Pfc::evaluateAction(const CmdVel& cmdVel, std::vector<Particle>& particles)
{
  // 行動価値
  double reward = 0.0;
  double postValue = 0.0;
  double actionValue = 0.0;
  double pfcValue = 0.0;
  for (Particle& particle : particles)
  {
    Pose pose = particle.pose_;

    // 次の状態の価値
    postValue = state_.getPosteriorValue(pose, cmdVel);
    // 状態遷移の報酬
    reward = state_.getReward(pose, cmdVel);
    // 回避重み
    particle.addAvoidanceWeightCandidate(reward);
    double avoidWeight = particle.getAvoidanceWeight();

    // 行動価値
    actionValue = -std::pow(abs(postValue), avoidWeight) + reward;
    // actionValue = postValue + reward;

    pfcValue += actionValue / std::pow(abs(state_.getValue(pose)), magnitude_);
    // pfcValue += actionValue / std::pow(abs(state_.getValue(pose)), magnitude_-avoidWeight);
    // pfcValue += actionValue / std::pow(abs(state_.getValue(pose)), 0);
  }
  return pfcValue;
}


// 意思決定（決定論的）
//------------------------------------------------------------------------------
CmdVel Pfc::decisionMaking(Pose& pose, double dt)
{
  // 行動価値のリスト
  std::vector<double> actionValues(cmdVels_.size());
  for (int i = 0; i < cmdVels_.size(); i++)
  {
    actionValues[i] = evaluateAction(cmdVels_[i], pose);
    // std::cout << actionValues[i] << std::endl;
  }

  // argmax
  std::vector<double>::iterator maxIterator = std::max_element(actionValues.begin(), actionValues.end());
  int maxIndex = std::distance(actionValues.begin(), maxIterator);
  // std::cout << maxIndex << std::endl;
  // std::cout << std::endl;

  return cmdVels_[maxIndex];
}


// 行動の評価（Q-PFCの計算）
//------------------------------------------------------------------------------
double Pfc::evaluateAction(const CmdVel& cmdVel, Pose& pose)
{
  // 次の状態の価値
  double postValue = state_.getPosteriorValue(pose, cmdVel);
  // 状態遷移の報酬
  double reward = state_.getReward(pose, cmdVel);

  // 行動価値
  double actionValue = postValue + reward;

  return actionValue;
}
