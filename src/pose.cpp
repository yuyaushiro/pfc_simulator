#include "pose.h"


// コンストラクタ
//------------------------------------------------------------------------------
Pose::Pose()
  : x(0)
  , y(0)
  , theta(0)
{}


// コンストラクタ
//------------------------------------------------------------------------------
Pose::Pose(double initialX, double initialY, double initialTheta)
  : x(initialX)
  , y(initialY)
  , theta(initialTheta)
{}


// +演算子オーバーロード
// ------------------------------------------------------------------------------



// +演算子オーバーロード
// ------------------------------------------------------------------------------
Pose Pose::operator+(const Pose& pose) const
{
  Pose tmp(this->x + pose.x, this->y + pose.y, this->theta + pose.theta);
  return tmp;
}


// -演算子オーバーロード
// ------------------------------------------------------------------------------
Pose Pose::operator-(const Pose& pose) const
{
  Pose tmp(this->x - pose.x, this->y - pose.y, this->theta - pose.theta);
  return tmp;
}


// ストリームオーバーロード
// ------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const Pose& pose)
{
  os << pose.x << ", " << pose.y << ", " << pose.theta;
  return os;
}
