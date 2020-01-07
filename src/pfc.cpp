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
CmdVel Pfc::decisionMaking(const std::vector<Particle>& particles)
{
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
  std::cout << std::endl;

  CmdVel cmdVel = cmdVels_[maxIndex];
  return cmdVel;
}


// 行動の評価（Q-PFCの計算）
//------------------------------------------------------------------------------
double Pfc::evaluateAction(const CmdVel& cmdVel, const std::vector<Particle>& particles)
{
  double dt = 0.1;
  // 行動価値
  double reward = 0.0;
  double postValue = 0.0;
  double actionValue = 0.0;
  double pfcValue = 0.0;
  for (int i = 0; i < particles.size(); i++)
  {
    Pose pose = particles[i].pose_;
    // double value = ss_.getValue(pose);
    // Pose poseNext = Robot::transitionState(cmdVel, dt, pose);
    // double valueNext = ss_.getValue(poseNext);

    // actionValue += (valueNext + 1000*dt) / std::pow(value, magnitude_);
    // std::cout << ss_.getActionValue(pose, cmdVel) << std::endl;

    postValue = ss_.getPosteriorValue(pose, cmdVel);
    reward = ss_.getReward(pose, cmdVel);
    actionValue = postValue + reward;
    pfcValue += actionValue / std::pow(ss_.getValue(pose), magnitude_);
  }
  return pfcValue;
}
