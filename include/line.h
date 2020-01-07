#pragma once

#include "point.h"


/// 線
class Line
{
public:
  /// コンストラクタ
  Line();
  Line(Point &point1, Point &point2, int width);
  Line(Point &point1, double theta, double length, int width);

  /// 描画する
  void draw();

private:
  /// 点1
  Point point1_;

  /// 点2
  Point point2_;

  /// 線の幅
  int width_;
};
