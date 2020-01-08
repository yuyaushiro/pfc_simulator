#include "pfc.h"
#include "robot.h"


// コンストラクタ
//------------------------------------------------------------------------------
Pfc::Pfc()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Pfc::Pfc(const std::vector<CmdVel>& cmdVels, const StateSpace& ss, double magnitude)
  : cmdVels_(cmdVels)
  , ss_(ss)
  , magnitude_(magnitude)
{}


// 意思決定
//------------------------------------------------------------------------------
CmdVel Pfc::decisionMaking(std::vector<Particle>& particles, double dt)
{
  // 行動価値のリスト
  std::vector<double> actionValues(cmdVels_.size());
  for (int i = 0; i < cmdVels_.size(); i++)
  {
    actionValues[i] = evaluateAction(cmdVels_[i], particles);
    std::cout << actionValues[i] << std::endl;
  }

  // argmax
  std::vector<double>::iterator maxIterator = std::max_element(actionValues.begin(), actionValues.end());
  int maxIndex = std::distance(actionValues.begin(), maxIterator);
  // std::cout << maxIndex << std::endl;
  std::cout << std::endl;

  // 回避重みを更新する
  for (int i = 0; i < particles.size(); i++)
  {
    particles[i].decreaseAvoidanceWeight(dt);
    particles[i].updateAvoidanceWeight(maxIndex);
  }

  CmdVel cmdVel = cmdVels_[maxIndex];
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
  for (int i = 0; i < particles.size(); i++)
  {
    Pose pose = particles[i].pose_;

    // 次の状態の価値
    postValue = ss_.getPosteriorValue(pose, cmdVel);
    // 状態遷移の報酬
    reward = ss_.getReward(pose, cmdVel);
    // 行動価値
    actionValue = postValue + reward;

    // 回避重み
    particles[i].addAvoidanceWeightCandidate(reward);
    double avoidWeight = particles[i].getAvoidanceWeight();

    pfcValue += actionValue / std::pow(abs(ss_.getValue(pose)), magnitude_ - avoidWeight);
    // pfcValue += actionValue / std::pow(abs(ss_.getValue(pose)), magnitude_);
    // pfcValue += actionValue;
  }
  return pfcValue;
}
