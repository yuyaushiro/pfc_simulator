#pragma once

#include <GLFW/glfw3.h>


/// 2次元の座標
class Point
{
public:
  /// コンストラクタ
  Point();
  Point(double x, double y);

  /// x座標
  GLfloat x;

  /// y座標
  GLfloat y;
};
