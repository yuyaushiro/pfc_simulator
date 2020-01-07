#pragma once

#include <string>
#include <vector>

#include "cmd_vel.h"
#include "pose.h"


/// 状態遷移
typedef struct Transition_
{
  /// 確率
  double probability = 0;

  /// 後の状態
  unsigned long posteriorState = 0;

  /// 報酬
  unsigned long reward = 0;
} Transition;


/// 行動
typedef struct Action_
{
  /// 状態遷移候補
  std::vector<Transition> transitions;
} Action;


/// 状態
class State
{
public:
  /// コンストラクタ
  State();

  /// コンストラクタ
  State(double value, std::vector<Action>& actions);

  /// 価値を取得
  double getValue() const;

  /// 状態遷移を取得
  std::vector<Transition> getTransitions(int actionIndex) const;

  /// 行動の追加
  void addTransition(int actionIndex, const Transition& transition);

private:
  /// 価値
  double value_;

  /// 行動
  std::vector<Action> actions_;
};
