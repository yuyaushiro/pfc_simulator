#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <cmath>

#include "goal.h"
#include "circle.h"
#include "line.h"


// コンストラクタ
//------------------------------------------------------------------------------
Goal::Goal()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Goal::Goal(const Pose& pose, double radius)
  : pose_(pose)
  , radius_(radius)
{}


// コンストラクタ
//------------------------------------------------------------------------------
bool Goal::inside(const Pose& pose) const
{
  double xDist = pose_.x - pose.x;
  double yDist = pose_.y - pose.y;
  return radius_ > sqrt(pow(xDist, 2) + pow(yDist, 2));
}


// 描画
//------------------------------------------------------------------------------
void Goal::draw()
{
  Circle region(0, 0, radius_, 1);
  Point point1(0, 0);
  GLfloat poleLength = 0.3;
  GLfloat flagUnder = 0.2;
  Line pole(point1, M_PI/2, (double)poleLength, 1);
  GLfloat const vtx[] = {0, flagUnder, 0, poleLength, 0.1, (flagUnder+poleLength)/2};
  glPushMatrix();
    glTranslated(pose_.x, pose_.y, 0.0);
    region.draw();
    pole.draw();
    // 三角形
    glVertexPointer(2, GL_FLOAT, 0, vtx);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColor4f(1.0f, 0.0f, 0.0f, 0.5f);
    glDrawArrays(GL_POLYGON, 0, 3);
    glDisableClientState(GL_VERTEX_ARRAY);
  glPopMatrix();
}
