#include <random>
#include <iostream>

#include "mcl.h"


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl(const Pose& initialPose, int particleNum)
  : particleNum_(particleNum)
{
  // 乱数生成
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<double> x_dist(initialPose.x, 0.1);
  std::normal_distribution<double> y_dist(initialPose.y, 0.1);
  std::normal_distribution<double> theta_dist(initialPose.theta, 0.05);

  // パーティクルの初期化
  particles_.resize(particleNum_);
  for (int i = 0; i < particleNum_; i++)
  {
    Pose pose(x_dist(mt), y_dist(mt), theta_dist(mt));
    double weight = 1.0/particleNum_;
    Avoidance avoidance(0.0, 3.0, 10.0);
    Particle p(pose, weight, avoidance);
    particles_[i] = p;
  }
}


// 動作による更新
//------------------------------------------------------------------------------
void Mcl::updateWithMotion(const CmdVel& cmdVel, double dt)
{
  //乱数生成クラス
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> normal(0, 0.01);

  for (int i = 0; i < particleNum_; i++)
  {
    // 乱数生成j
    std::vector<double> ns(4);
    for (int j = 0; j < 4; j++)
    {
      ns[j] = normal(mt);
    }
    particles_[i].updateWithMotion(cmdVel, dt, ns);
  }
}


// ゴール観測によるアップデート
//------------------------------------------------------------------------------
void Mcl::updateWithGoalObservation(const Goal& goal)
{
  for (int i = 0; i < particleNum_; i++)
  {
    particles_[i].updateWithGoalObservation(goal);
  }
}


// リサンプリング
//------------------------------------------------------------------------------
void Mcl::resampling()
{
  double weightSum = 0.0;
  // 重み合計
  for (int i = 0; i < particleNum_; i++)
    weightSum += particles_[i].getWeight();

  //乱数生成クラス
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> dist(0, weightSum/particleNum_);

  std::vector<Particle> newParticles(particleNum_);

  double r = dist(mt);
  double c = particles_[0].getWeight();
  double i = 0;

  double U;
  for (int m = 0; m < particleNum_; m++)
  {
    U = r + m * weightSum/particleNum_;
    while (U > c)
    {
      i = i + 1;
      c = c + particles_[i].getWeight();
    }
    newParticles[m] = particles_[i];
  }
  particles_ = newParticles;
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
