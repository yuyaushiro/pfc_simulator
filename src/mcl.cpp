#include <random>
#include <iostream>

#include "mcl.h"


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Mcl::Mcl(const Pose& initialPose, int particleNum, int seedValue,
         const std::vector<double> initPoseStd, const std::vector<double> motionStd)
  : particleNum_(particleNum)
  , mt_(seedValue)
  , motionStd_(motionStd)
{
  init(initialPose, initPoseStd);
}


// 初期化
//------------------------------------------------------------------------------
void Mcl::init(const Pose& initialPose, const std::vector<double> initPoseStd)
{
  std::normal_distribution<double> xDist(initialPose.x, initPoseStd[0]);
  std::normal_distribution<double> yDist(initialPose.y, initPoseStd[1]);
  std::normal_distribution<double> thetaDist(initialPose.theta, initPoseStd[2]);

  // パーティクルの初期化
  particles_.resize(particleNum_);
  for (Particle& particle : particles_)
  {
    Pose pose(xDist(mt_), yDist(mt_), thetaDist(mt_));
    double weight = 1.0/particleNum_;
    Avoidance avoidance(0.0, 3.0, 10.0);
    Particle p(pose, weight, avoidance);
    particle = p;
  }
}


// 動作による更新
//------------------------------------------------------------------------------
void Mcl::updateWithMotion(const CmdVel& cmdVel, double dt)
{
  //乱数生成クラス
  std::normal_distribution<> nuDist(0, motionStd_[0]);
  std::normal_distribution<> omegaDist(0, motionStd_[1]);

  for (Particle& particle : particles_)
  {
    // 乱数生成
    std::vector<double> ns{nuDist(mt_), 0.0, 0.0, omegaDist(mt_)};

    particle.updateWithMotion(cmdVel, dt, ns);
  }
}


// ゴール観測によるアップデート
//------------------------------------------------------------------------------
void Mcl::updateWithGoalObservation(const Goal& goal)
{
  for (Particle& particle : particles_)
  {
    particle.updateWithGoalObservation(goal);
  }
}


// リサンプリング
//------------------------------------------------------------------------------
void Mcl::resampling()
{
  double weightSum = 0.0;
  // 重み合計
  for (Particle& particle : particles_)
    weightSum += particle.getWeight();

  //乱数生成クラス
  std::uniform_real_distribution<double> dist(0, weightSum/particleNum_);

  std::vector<Particle> newParticles(particleNum_);

  double r = dist(mt_);
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
  for (Particle& particle : particles_)
  {
    particle.draw();
  }
}
