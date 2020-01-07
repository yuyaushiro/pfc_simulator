#include <cmath>
#include <GL/glew.h>

#include "circle.h"


// コンストラクタ
//------------------------------------------------------------------------------
Circle::Circle()
  : x_(0)
  , y_(0)
  , radius_(1)
  , width_(1)
{}


// コンストラクタ
//------------------------------------------------------------------------------
Circle::Circle(double x, double y, double radius, int width)
  : x_(x)
  , y_(y)
  , radius_(radius)
  , width_(width)
{}


// 描画
//------------------------------------------------------------------------------
void Circle::draw()
{
  GLfloat vtx[36*2];
  for(int i = 0; i < 36; ++i){
    GLfloat angle = static_cast<GLfloat>((M_PI*2.0*i)/36);
    vtx[i*2]   = radius_ * std::sin(angle);
    vtx[i*2+1] = radius_ * std::cos(angle);
  }
  glVertexPointer(2, GL_FLOAT, 0, vtx);

  glEnableClientState(GL_VERTEX_ARRAY);
  glColor3f(1.0f, 1.0f, 1.0f);
  glDrawArrays(GL_POLYGON, 0, 36);
  glDisableClientState(GL_VERTEX_ARRAY);

  glEnableClientState(GL_VERTEX_ARRAY);
  glColor3f(0.0f, 0.0f, 0.0f);
  glLineWidth(width_);
  glDrawArrays(GL_LINE_LOOP, 0, 36);
  glDisableClientState(GL_VERTEX_ARRAY);
}
