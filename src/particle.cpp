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
Particle::Particle(Pose& initialPose, double weight)
  : pose_(initialPose)
  , weight_(weight)
  , radius_(0.02)
{}


// 動作による更新
//------------------------------------------------------------------------------
void Particle::transitionStateWithNoise(CmdVel& cmdVel, double dt, const std::vector<double>& ns)
{
  double pnu = cmdVel.nu + ns[0]*sqrt(abs(cmdVel.nu)/dt) + ns[1]*sqrt(abs(cmdVel.omega)/dt);
  double pomega = cmdVel.omega + ns[2]*sqrt(abs(cmdVel.nu)/dt) + ns[3]*sqrt(abs(cmdVel.omega)/dt);

  // ノイズを含んだ行動
  CmdVel noisyCmdVel(pnu, pomega, "");
  pose_ = Robot::transitionState(noisyCmdVel, dt, pose_);
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
    glColor4f(0.5f, 0.5f, 1.0f, 0.5f);
    rotation.draw();
    // 位置の描画
    glColor4f(0.0f, 0.0f, 1.0f, 0.5f);
    glRectf(-radius_, -radius_, radius_, radius_);
  glPopMatrix();
}
