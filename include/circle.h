#pragma once


/// 円
class Circle
{
public:
  /// コンストラクタ
  Circle();
  Circle(double x, double y, double radius, int width);

  /// 描画する
  void draw();

private:
  // 中心座標
  /// x座標
  double x_;
  /// y座標
  double y_;

  /// 半径
  double radius_;

  /// 線の幅
  int width_;
};
