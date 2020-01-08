#pragma once

#include "pose.h"


/// ゴール
class Goal
{
public:
  /// コンストラクタ
  Goal();

  /// コンストラクタ
  Goal(const Pose& pose, double radius);

  /// ゴールに入っているか判定
  bool inside(const Pose& pose);

  /// 描画
  void draw();

private:
  /// 姿勢
  Pose pose_;

  /// 半径
  double radius_;
};
