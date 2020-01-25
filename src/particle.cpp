#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <GLFW/glfw3.h>

#include "particle.h"
#include "robot.h"
#include "circle.h"
#include "point.h"
#include "line.h"


// コンストラクタ
//------------------------------------------------------------------------------
Particle::Particle()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Particle::Particle(const Pose& initialPose, double weight, const Avoidance& avoidance)
  : pose_(initialPose)
  , weight_(weight)
  , radius_(0.02)
  , avoidance_(avoidance)
{}


// 動作による更新
//------------------------------------------------------------------------------
void Particle::updateWithMotion(const CmdVel& cmdVel, double dt, const std::vector<double>& ns)
{
  double pnu = cmdVel.nu + ns[0]*sqrt(abs(cmdVel.nu)/dt) + ns[1]*sqrt(abs(cmdVel.omega)/dt);
  double pomega = cmdVel.omega + ns[2]*sqrt(abs(cmdVel.nu)/dt) + ns[3]*sqrt(abs(cmdVel.omega)/dt);

  // ノイズを含んだ行動
  CmdVel noisyCmdVel(pnu, pomega, "");
  pose_ = Robot::transitionState(noisyCmdVel, dt, pose_);
}


// ゴール観測による更新
//------------------------------------------------------------------------------
void Particle::updateWithGoalObservation(const Goal& goal)
{
  if (goal.inside(pose_))
    weight_ *= 1e-10;
}


// 重みの取得
//------------------------------------------------------------------------------
double Particle::getWeight()
{
  return weight_;
}


// 回避重みの取得
//------------------------------------------------------------------------------
double Particle::getAvoidanceWeight()
{
  return avoidance_.getWeight();
}


// 回避重みの候補を追加
//------------------------------------------------------------------------------
void Particle::addAvoidanceWeightCandidate(double reward)
{
  avoidance_.addWeightCandidate(reward);
}


// 回避重みの更新
//------------------------------------------------------------------------------
void Particle::updateAvoidanceWeight(int candidateIndex)
{
  avoidance_.updateWeight(candidateIndex);
}


// 重みの減少
//------------------------------------------------------------------------------
void Particle::decreaseAvoidanceWeight(double dt)
{
  avoidance_.decreaseWeight(dt);
}


// 描画
//------------------------------------------------------------------------------
void Particle::draw()
{
  Point point1(0, 0);
  Line rotation(point1, 0, radius_+0.1, 3);

  glPushMatrix();
    glTranslated(pose_.x, pose_.y, 0.0);
    glRotated(pose_.theta*180/M_PI, 0, 0, 1);
    // 方向の描画
    (avoidance_.getWeight() == 1.0) ?
      glColor4f(0.5f, 0.5f, 1.0f, 0.5f) :
      glColor4f(1.0f, 0.5f, 0.5f, 0.5f);
    rotation.draw();
    // 位置の描画
    (avoidance_.getWeight() == 1.0) ?
      glColor4f(0.0f, 0.0f, 1.0f, 0.5f) :
      glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
    glRectf(-radius_, -radius_, radius_, radius_);
  glPopMatrix();
}
