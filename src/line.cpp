#include <cmath>
#include <iostream>
#include <GL/glew.h>

#include "line.h"


// コンストラクタ
//------------------------------------------------------------------------------
Line::Line()
  : point1_()
  , point2_()
  , width_(1)
{}


// コンストラクタ
//------------------------------------------------------------------------------
Line::Line(Point &point1, Point &point2, int width)
  : point1_(point1)
  , point2_(point2)
  , width_(width)
{}


// コンストラクタ
//------------------------------------------------------------------------------
Line::Line(Point &point1, double theta, double length, int width)
  : point1_(point1)
  , point2_(length*cos(theta), length*sin(theta))
  , width_(width)
{
}


// 描画
//------------------------------------------------------------------------------
void Line::draw()
{
  GLfloat const vtx[] = {point1_.x, point1_.y, point2_.x, point2_.y};
  glVertexPointer(2, GL_FLOAT, 0, vtx);

  glLineWidth(width_);
  glEnableClientState(GL_VERTEX_ARRAY);
  glDrawArrays(GL_LINE_LOOP, 0, 2);
  glDisableClientState(GL_VERTEX_ARRAY);
}
