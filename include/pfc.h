#pragma once

#include <vector>

#include "mcl.h"
#include "particle.h"
#include "state.h"
#include "cmd_vel.h"
#include "state.h"


class Pfc
{
public:
  /// コンストラクタ
  Pfc();
  Pfc(const std::vector<CmdVel>& cmdVels, const State& state, double magnitude);

  /// デストラクタ
  ~Pfc() = default;

  /// 意思決定
  CmdVel decisionMaking(std::vector<Particle>& particles, double dt);

private:
  /// 行動の評価（Q-PFCの計算）
  double evaluateAction(const CmdVel& cmdVel, std::vector<Particle>& particles);

private:
  /// 行動リスト
  std::vector<CmdVel> cmdVels_;

  /// 状態
  State state_;

  /// 分母指数部
  double magnitude_;

  /// 行動の履歴
  std::vector<CmdVel> history_;
};
