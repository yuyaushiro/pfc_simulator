#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <iostream>
#include <GLFW/glfw3.h>

#include "window.h"
#include "goal.h"
#include "grid_map.h"
#include "mcl.h"
#include "robot.h"
#include "saver.h"


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

  // std::vector<CmdVel> cmdVels{CmdVel(0.1, 0.0, "fw"), CmdVel(0.0, 0.5, "ccw"), CmdVel(0.0, -0.5, "cw")};
  std::vector<CmdVel> cmdVels{CmdVel(0.2, 0.0, "fw"), CmdVel(0.0, 1.0, "ccw"), CmdVel(0.0, -1.0, "cw")};

  // Pose minPose(-2.5, -2.5, 0);
  // Goal goal(Pose(3.75, 3.75, 0)+minPose, 0.1);
  // GridMap gridMap(std::string("GimpCorner_100x100"), std::vector<double>{0.05, 0.05}, minPose);
  // State state(std::string("GimpCorner_100x100x36"), gridMap, cmdVels,
  //             std::vector<int>{100, 100, 36}, std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose);

  Pose minPose(-5.0, -5.0, 0);
  Goal goal(Pose(6.0, 7.5, 0)+minPose, 0.15);
  GridMap gridMap(std::string("Gimp1Corner_200x200"), std::vector<double>{0.05, 0.05}, minPose);
  State state(std::string("Gimp1Corner_200x200x36"), gridMap, cmdVels,
              std::vector<int>{200, 200, 36}, std::vector<double>{0.05, 0.05, M_PI/18.0}, minPose);

  // 初期姿勢
  Pose initPose(-3.0, -3.0, 0);
  // 初期姿勢のばらつき
  std::vector<double> initPoseStd{0.3, 0.3, 0.03};
  // 動作のばらつき
  std::vector<double> motionStd{0.01, 0.01};
  // 乱数シード
  std::random_device rnd;
  // 自己位置推定
  Mcl mcl(initPose, 500, rnd(), initPoseStd, motionStd);
  // エージェント
  Pfc pfc(cmdVels, state, 2.0);
  // ロボット
  Robot robot(initPose, goal, mcl, pfc, rnd(), motionStd);
  robot.restart(initPose, initPoseStd);

  // セーブ
  Saver saver(std::string("q-mdp"), gridMap, goal);

  double prevTime = glfwGetTime();
  // 描画のループ
  while (window && saver.trialNum_ <= 100)
  {
    double currentTime = glfwGetTime();
    double elapsedTime = currentTime - prevTime;
    // robot.oneStep(elapsedTime);
    // std::cout << elapsedTime << std::endl;
    robot.oneStep(0.1);

    // バッファのクリア
    glClearColor(0.9f, 0.9f, 0.9f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // 表示
    gridMap.draw(abs(minPose.x));
    goal.draw();
    robot.draw();


    saver.saveOneStep(robot.getParticles());
    // ゴールしたら
    if (goal.inside(robot.getPose()))
    {
      saver.saveOneTrial(true);
      robot.restart(initPose, initPoseStd);
      // std::cout << "ゴール" << std::endl;
    }
    // 脱輪したら
    if (gridMap.insideObstacle(robot.getPose()) || saver.elapsedTime_ >= 3000)
    // if (gridMap.insideObstacle(robot.getPose()) || saver.elapsedTime_ >= 1200)
    {
      saver.saveOneTrial(false);
      robot.restart(initPose, initPoseStd);
      // std::cout << "脱輪" << std::endl;
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
