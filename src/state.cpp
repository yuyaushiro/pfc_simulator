#include <cmath>
#include <fstream>
#include <sstream>

#include "state.h"


// コンストラクタ
//------------------------------------------------------------------------------
State::State()
{}


// コンストラクタ
//------------------------------------------------------------------------------
State::State(double value, std::vector<Action>& actions)
  : value_(value)
  , actions_(actions)
{}


// 価値の取得
//------------------------------------------------------------------------------
double State::getValue() const
{
  return value_;
}


// 状態遷移を取得
//------------------------------------------------------------------------------
std::vector<Transition> State::getTransitions(int actionIndex) const
{
  return actions_[actionIndex].transitions;
}


// 行動の追加
//------------------------------------------------------------------------------
void State::addTransition(int actionIndex, const Transition& transition)
{
  actions_[actionIndex].transitions.push_back(transition);
}
