#pragma once

#include "pose.h"


/// ゴール
class Goal
{
public:
  /// コンストラクタ
  Goal();
  Goal(const Pose& pose, double radius);

  /// 描画
  void draw();

private:
  /// 姿勢
  Pose pose_;

  /// 半径
  double radius_;
};
