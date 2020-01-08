#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "simulator.h"


// コンストラクタ
//------------------------------------------------------------------------------
Simulator::Simulator()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Simulator::Simulator(const Window& window, const Robot& robot, const Goal& goal)
  : window_(window)
  , robot_(robot)
  , goal_(goal)
{}


// 動作
//------------------------------------------------------------------------------
void Simulator::run(bool display)
{
  Pose initPose(1, -1, M_PI/2);
  double prevTime = glfwGetTime();
  while (window_)
  {
    // 描画
    if (display)
    {
      double currentTime = glfwGetTime();
      double elapsedTime = currentTime - prevTime;

      robot_.oneStep(elapsedTime);

      // バッファのクリア
      glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
      glClear(GL_COLOR_BUFFER_BIT);

      goal_.draw();
      robot_.draw();

      // ダブルバッファのスワップ
      window_.swapBuffers();
      glfwPollEvents();

      prevTime = currentTime;
    }
    else
    {
      robot_.oneStep(0.1);
    }

    // ロボットがゴールした
    if (goal_.inside(robot_.getPose()))
    {
      std::cout << "Goal" << std::endl;
      robot_.restart(initPose);
    }
  }
}
