#include "saver.h"


// コンストラクタ
//------------------------------------------------------------------------------
Saver::Saver()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Saver::Saver(const std::string& saveFileName, const GridMap& gridMap,
             const Goal& goal)
  : saveFileName_("../save/" + saveFileName + ".csv")
  , ofs_(saveFileName_)
  , gridMap_(gridMap)
  , elapsedTime_(0)
  , particlesInObstacle_(0)
  , trialNum_(1)
{
}


// 1ステップセーブ
//------------------------------------------------------------------------------
void Saver::saveOneStep(const std::vector<Particle>& particles)
{
  elapsedTime_++;
  int particlesInObstacleNum = 0;
  for (int i = 0; i < particles.size(); i++)
  {
    if (gridMap_.insideObstacle(particles[i].pose_))
    {
      particlesInObstacleNum++;
    }
  }
  particlesInObstacle_ += particlesInObstacleNum;
}


// 1試行セーブ
//------------------------------------------------------------------------------
void Saver::saveOneTrial(bool goal)
{
  ofs_ << trialNum_ << ", " << goal << ", " << elapsedTime_*0.1 << ", " << particlesInObstacle_*0.1 << std::endl;
  std::cout << trialNum_ << ", " << goal << ", " << elapsedTime_*0.1 << ", " << particlesInObstacle_*0.1 << std::endl;
  elapsedTime_ = 0;
  particlesInObstacle_ = 0;
  trialNum_++;
}


// 状況の書き込み
//------------------------------------------------------------------------------
void Saver::writeToFile()
{
  writeLine(std::string("aaa"));
}


// 一行書き込み
//------------------------------------------------------------------------------
void Saver::writeLine(std::string oneLine)
{
  ofs_ << oneLine << std::endl;
}
