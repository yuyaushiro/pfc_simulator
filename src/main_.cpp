#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <iostream>
#include <GLFW/glfw3.h>

#include "window.h"
// #include "state_space.h"
#include "goal.h"
// #include "robot.h"
#include "pfc.h"
#include "mcl.h"
#include "map.h"


int main(int argc, const char *argv[])
{
  // GLFWの初期化
  if (!glfwInit())
  {
    // 初期化に失敗
    std::cerr << "Can't initialize GLFW." << std::endl;
    exit(EXIT_FAILURE);
  }

  // ウィンドウの生成
  Window window;

  std::vector<CmdVel> cmdVels{CmdVel(0.1, 0.0, "fw"), CmdVel(0.0, 0.5, "ccw"), CmdVel(0.0, -0.5, "cw")};

  // Pose minPose(-5.0, -5.0, 0);
  // Goal goal(Pose(6.75, 8.0, 0)+minPose, 0.15);
  // StateSpace ss(std::string("CorridorGimp_200x200x36"), std::vector<int>{200, 200, 36},
  //               std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose, cmdVel);

  Pose minPose(-2.5, -2.5, 0);
  // Goal goal(Pose(6.75/2, 4.0, 0)+minPose, 0.15);
  // StateSpace ss(std::string("CorridorGimp_100x100x36"), std::vector<int>{100, 100, 36},
  //               std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose, cmdVels);


  // Pose initPose(-1, -1, M_PI/2);
  // Mcl mcl(initPose, 1000);
  // Pfc pfc(cmdVels, ss, 2.0);
  // Robot robot(initPose, goal, mcl, pfc);

  Map map("CorridorGimp_100x100.png", minPose, std::vector<double>{0.05, 0.05});

  double prevTime = glfwGetTime();
  // 描画のループ
  while (window)
  {
    double currentTime = glfwGetTime();
    double elapsedTime = currentTime - prevTime;

    // robot.oneStep(0.1);
    // robot.oneStep(elapsedTime);

    // バッファのクリア
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // goal.draw();
    // robot.draw();

    // if (goal.inside(robot.getPose()))
    //   std::cout << "Goal" << std::endl;

    // ダブルバッファのスワップ
    window.swapBuffers();
    glfwPollEvents();

    prevTime = currentTime;
  }

  // GLFWの終了処理
  glfwTerminate();

  return 0;
}
