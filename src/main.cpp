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

  Pose minPose(-2.5, -2.5, 0);
  Goal goal(Pose(3.75, 3.75, 0)+minPose, 0.15);
  GridMap gridMap(std::string("Gimp2Corner_100x100"), std::vector<double>{0.05, 0.05}, minPose);
  State state(std::string("Gimp2Corner_100x100x36"), gridMap, cmdVels,
              std::vector<int>{100, 100, 36}, std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose);

  // Pose minPose(-5.0, -5.0, 0);
  // Goal goal(Pose(7.5, 7.5, 0)+minPose, 0.15);
  // GridMap gridMap(std::string("Gimp2Corner_200x200"), std::vector<double>{0.05, 0.05}, minPose);
  // State state(std::string("Gimp2Corner_200x200x36"), gridMap, cmdVels,
  //             std::vector<int>{200, 200, 36}, std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose);

  Pose initPose(-1.75, 0.75, 0);
  std::vector<double> initPoseStd{0.1, 0.1, 0.05};
  std::vector<double> motionStd{0.02, 0.02};
  std::random_device rnd;
  Mcl mcl(initPose, 1000, rnd(), initPoseStd, motionStd);
  Pfc pfc(cmdVels, state, 1.0);
  Robot robot(initPose, goal, mcl, pfc);

  double prevTime = glfwGetTime();
  // 描画のループ
  while (window)
  {
    double currentTime = glfwGetTime();
    double elapsedTime = currentTime - prevTime;

    // robot.oneStep(elapsedTime);
    robot.oneStep(0.1);

    // バッファのクリア
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    gridMap.draw(abs(minPose.x));
    goal.draw();
    robot.draw();

    // ゴールしたら
    if (goal.inside(robot.getPose()))
    {
      // robot.restart(initPose, initPoseStd);
      std::cout << "ゴール" << std::endl;
    }
    // 脱輪したら
    if (gridMap.insideObstacle(robot.getPose()))
    {
      robot.restart(initPose, initPoseStd);
      std::cout << "脱輪" << std::endl;
    }

    // ダブルバッファのスワップ
    window.swapBuffers();
    glfwPollEvents();

    prevTime = currentTime;
  }

  // GLFWの終了処理
  glfwTerminate();

  return 0;
}
