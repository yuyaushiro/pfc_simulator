#include <random>
#include <iostream>

#include "mcl.h"


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl(Pose& initialPose, int particleNum)
  : particleNum_(particleNum)
{
  // 乱数生成
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<double> x_dist(initialPose.x, 0.1);
  std::normal_distribution<double> y_dist(initialPose.y, 0.1);
  std::normal_distribution<double> theta_dist(initialPose.theta, 0.1);

  // パーティクルの初期化
  particles_.resize(particleNum_);
  for (int i = 0; i < particleNum_; i++)
  {
    Pose pose(x_dist(mt), y_dist(mt), theta_dist(mt));
    Particle p(pose, 1/particleNum_);
    particles_[i] = p;
  }
}


// 動作による更新
//------------------------------------------------------------------------------
void Mcl::updateWithMotion(CmdVel& cmdVel, double dt)
{
  //乱数生成クラス
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> normal(0, 0.03);

  for (int i = 0; i < particleNum_; i++)
  {
    // 乱数生成
    std::vector<double> ns(4);
    for (int j = 0; j < 4; j++)
    {
      ns[j] = normal(mt);
    }
    particles_[i].transitionStateWithNoise(cmdVel, dt, ns);
  }
  std::cout << "---" << std::endl;
}


// 描画
//------------------------------------------------------------------------------
void Mcl::draw()
{
  for (int i = 0; i < particleNum_; i++)
  {
    particles_[i].draw();
  }
}
