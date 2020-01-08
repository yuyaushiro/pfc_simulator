#pragma once

#include "goal.h"
#include "robot.h"
#include "window.h"


class Simulator
{
public:
  /// コンストラクタ
  Simulator();

  /// コンストラクタ
  Simulator(const Window& window, const Robot& robot, const Goal& goal);

  /// 動作
  void run(bool display);

private:
  /// ウィンドウ
  Window window_;

  /// ロボット
  Robot robot_;

  /// ゴール
  Goal goal_;
};
