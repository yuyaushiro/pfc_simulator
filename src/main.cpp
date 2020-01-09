#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <iostream>
#include <GLFW/glfw3.h>

#include "window.h"
#include "state_space.h"
#include "goal.h"
#include "robot.h"
#include "pfc.h"
#include "mcl.h"
#include "simulator.h"


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

  // Pose minPose(-2.5, -2.5, 0);
  // Goal goal(Pose(6.75/2, 4.0, 0)+minPose, 0.15);
  // StateSpace ss(std::string("CorridorGimp_100x100x18"), std::vector<int>{100, 100, 18},
  //               std::vector<double>{0.05, 0.05, M_PI/9.0}, minPose, cmdVels);

  Pose minPose(-2.5, -2.5, 0);
  Goal goal(Pose(6.75/2, 4.0, 0)+minPose, 0.2);
  StateSpace ss(std::string("Gimp2Corner_100x100x18"), std::vector<int>{100, 100, 18},
                std::vector<double>{0.05, 0.05, M_PI/9.0}, minPose, cmdVels);

  Pose initPose(-1.75, -1.5, M_PI/2);
  Mcl mcl(initPose, 1000);
  Pfc pfc(cmdVels, ss, 2.0);
  Robot robot(initPose, goal, mcl, pfc);

  // シミュレーション
  Simulator simulator(window, robot, goal);
  simulator.run(true);

  // GLFWの終了処理
  glfwTerminate();

  return 0;
}
