#pragma once

#include <iostream>


/// 二次元の位置と方向
class Pose
{
public:
  /// コンストラクタ
  Pose();
  Pose(double x, double y, double theta);

  /// +演算子オーバーロード
  Pose operator+(const Pose& pose) const;

  /// -演算子オーバーロード
  Pose operator-(const Pose& pose) const;

  /// ストリームオーバーロード
  friend std::ostream& operator<<(std::ostream& os, const Pose& pose);

  /// x座標
  double x;

  /// y座標
  double y;

  /// 方向
  double theta;
};
