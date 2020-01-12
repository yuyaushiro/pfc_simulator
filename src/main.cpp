#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <iostream>
#include <GLFW/glfw3.h>

#include "window.h"
#include "goal.h"
#include "grid_map.h"
#include "mcl.h"
#include "robot.h"


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

  Pose minPose(-5.0, -5.0, 0);
  Goal goal(Pose(7.0, 8.0, 0)+minPose, 0.15);
  GridMap gridMap(std::string("CorridorGimp_200x200"), std::vector<double>{0.05, 0.05}, minPose);
  State state(std::string("CorridorGimp_200x200x36"), gridMap, cmdVels,
              std::vector<int>{200, 200, 36}, std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose);

  // Pose minPose(-2.5, -2.5, 0);
  // Goal goal(Pose(3.5, 4.0, 0)+minPose, 0.15);
  // GridMap gridMap(std::string("CorridorGimp_100x100"), minPose, std::vector<double>{0.05, 0.05});
  // State state(std::string("CorridorGimp_100x100x18"), gridMap, cmdVels,
  //             std::vector<int>{100, 100, 18}, std::vector<double>{0.05, 0.05, M_PI/9.0}, minPose);

  Pose initPose(-1, -1, 0);
  Mcl mcl(initPose, 1000);
  Pfc pfc(cmdVels, state, 2.0);
  Robot robot(initPose, goal, mcl, pfc);


  double prevTime = glfwGetTime();
  // 描画のループ
  while (window)
  {
    double currentTime = glfwGetTime();
    double elapsedTime = currentTime - prevTime;

    robot.oneStep(elapsedTime);

    // バッファのクリア
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gridMap.draw(abs(minPose.x));
    goal.draw();
    robot.draw();

    // ダブルバッファのスワップ
    window.swapBuffers();
    glfwPollEvents();

    prevTime = currentTime;
  }

  // GLFWの終了処理
  glfwTerminate();

  return 0;
}
