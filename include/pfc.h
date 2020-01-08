#pragma once

#include <vector>

#include "mcl.h"
#include "particle.h"
#include "state_space.h"
#include "cmd_vel.h"


class Pfc
{
public:
  /// コンストラクタ
  Pfc();
  Pfc(const std::vector<CmdVel>& cmdVels, const StateSpace& ss, double magnitude);

  /// デストラクタ
  ~Pfc() = default;

  /// 意思決定
  CmdVel decisionMaking(std::vector<Particle>& particles, double dt);

  /// 行動の評価（Q-PFCの計算）
  double evaluateAction(const CmdVel& cmdVel, std::vector<Particle>& particles);

private:
  /// 行動リスト
  std::vector<CmdVel> cmdVels_;

  /// 状態
  StateSpace ss_;

  /// 分母指数部
  double magnitude_;
};
