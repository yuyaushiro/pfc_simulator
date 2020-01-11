#include <cmath>
#include <fstream>
#include <sstream>

#include "state.h"


// コンストラクタ
//------------------------------------------------------------------------------
State::State()
{}


// コンストラクタ
//------------------------------------------------------------------------------
State::State(const std::string& fileName, const GridMap& gridMap, const std::vector<CmdVel>& cmdVels,
             const std::vector<int>& cellNum, const std::vector<double>& cellWidth, const Pose& minPose)
  : valueFileName_("../value/" + fileName + ".value")
  , transProbFileName_("../stp/" + fileName + ".stp")
  , gridMap_(gridMap)
  , cmdVels_(cmdVels)
  , cellNum_(cellNum)
  , cellWidth_(cellWidth)
  , minPose_(minPose)
{
  loadValueFile();
  loadTransProbFile();
}


/// 価値を取得
//------------------------------------------------------------------------------
double State::getValue(const Pose& pose) const
{
  unsigned long stateIndex = toStateIndex(pose);
  double value = -( values_[stateIndex] * 0.001 );
  if (value == 0.0)
    value = -1e-10;
  return value;
}


/// 価値を取得
//------------------------------------------------------------------------------
double State::getPosteriorValue(const Pose& pose, const CmdVel& cmdVel) const
{
  double postValue = 0.0;
  int actionIndex = getActionIndex(cmdVel.name);
  int thetaIndex = toCellIndex(pose)[2];
  std::vector<Transition> transitions = transProb_[actionIndex][thetaIndex];
  for (int i = 0; i < transitions.size(); i++)
  {
    std::vector<int> delta = transitions[i].deltaTransCell;
    double prob = transitions[i].probability;
    // 遷移後の姿勢
    Pose afterPose =
      pose + Pose(delta[0]*cellWidth_[0], delta[1]*cellWidth_[1], delta[2]*cellWidth_[2]);

    // 遷移後の状態価値
    postValue += getValue(afterPose) * prob;
  }

  return postValue;
}


/// 価値を取得
//------------------------------------------------------------------------------
double State::getReward(const Pose& pose, const CmdVel& cmdVel) const
{
  double reward = 0.0;
  int actionIndex = getActionIndex(cmdVel.name);
  int thetaIndex = toCellIndex(pose)[2];
  std::vector<Transition> transitions = transProb_[actionIndex][thetaIndex];
  for (int i = 0; i < transitions.size(); i++)
  {
    std::vector<int> delta = transitions[i].deltaTransCell;
    double prob = transitions[i].probability;
    // 遷移後の姿勢
    Pose afterPose =
      pose + Pose(delta[0]*cellWidth_[0], delta[1]*cellWidth_[1], delta[2]*cellWidth_[2]);

    // 遷移後の状態価値
    reward += (1 + gridMap_.insideObstacle(afterPose)*1000) * 100 * prob;
  }

  return -( reward * 0.001 );
}


// 状態インデックスへの変換
//------------------------------------------------------------------------------
std::vector<int> State::toCellIndex(const Pose& pose) const
{
  std::vector<int> index(3);
  Pose poseInState = pose - minPose_;
  index[0] = static_cast<int>(floor(poseInState.x / cellWidth_[0]));
  index[1] = static_cast<int>(floor(poseInState.y / cellWidth_[1]));
  index[2] = static_cast<int>(floor(poseInState.theta / cellWidth_[2]));

  // 正規化
  for (int i = 0; i < 2; i++)
  {
    if (index[i] < 0)
      index[i] = 0;
    else if (index[i] >= cellNum_[i])
      index[i] = cellNum_[i] - 1;
  }
  index[2] = (index[2] + cellNum_[2]*1000) % cellNum_[2];

  return index;
}


// 状態インデックスへの変換
//------------------------------------------------------------------------------
unsigned long State::toStateIndex(const Pose& pose) const
{
  std::vector<int> index(3);
  Pose poseInState = pose - minPose_;
  index[0] = static_cast<int>(floor(poseInState.x / cellWidth_[0]));
  index[1] = static_cast<int>(floor(poseInState.y / cellWidth_[1]));
  index[2] = static_cast<int>(floor(poseInState.theta / cellWidth_[2]));

  // 正規化
  for (int i = 0; i < 2; i++)
  {
    if (index[i] < 0)
      index[i] = 0;
    else if (index[i] >= cellNum_[i])
      index[i] = cellNum_[i] - 1;
  }
  index[2] = (index[2] + cellNum_[2]*1000) % cellNum_[2];

  unsigned long state = index[0]*cellNum_[1]*cellNum_[2] + index[1]*cellNum_[2] + index[2];
  return state;
}


// 価値関数ファイルを読み込む
//------------------------------------------------------------------------------
void State::loadValueFile()
{
  // サイズ変更
  size_t stateNum = cellNum_[0] * cellNum_[1] * cellNum_[2];
  values_.resize(stateNum);

  // ファイル読み込み
  std::ifstream ifs(valueFileName_);
  std::string line;
  while (getline(ifs, line))
  {
    std::vector<std::string> words = splitOneLine(line, ' ');
    size_t index = std::stoul(words[0]);
    double value = std::stod(words[1]);
    values_[index] = value;
  }

  ifs.close();
}


// 遷移確率ファイルを読み込む
//------------------------------------------------------------------------------
void State::loadTransProbFile()
{
  // サイズ変更
  transProb_.resize(cmdVels_.size());
  for (int i = 0; i < transProb_.size(); i++)
  {
    transProb_[i].resize(cellNum_[2]);
  }

  // ファイル読み込み
  std::ifstream ifs(transProbFileName_);
  std::string line;
  while (getline(ifs, line))
  {
    std::vector<std::string> words = splitOneLine(line, ' ');
    size_t actionIndex = getActionIndex(words[0]);
    int thetaIndex = std::stoi(words[1]);
    double prob = std::stod(words[2]);
    std::vector<int> deltaTransCell{std::stoi(words[3]), std::stoi(words[4]), std::stoi(words[5])};
    Transition trans = {prob, deltaTransCell};
    transProb_[actionIndex][thetaIndex].push_back(trans);
    // std::cout << trans.probability << std::endl;
  }

  ifs.close();
}


// 1行を分割
//------------------------------------------------------------------------------
std::vector<std::string> State::splitOneLine(std::string& line, char delimiter) const
{
  std::istringstream stream(line);
  std::string field;
  std::vector<std::string> words;
  while (getline(stream, field, delimiter))
  {
    words.push_back(field);
  }

  return words;
}


// 行動のインデックスを取得
//------------------------------------------------------------------------------
int State::getActionIndex(const std::string& actionName) const
{
  for (int i = 0; i < cmdVels_.size(); i++)
  {
    if (cmdVels_[i].name == actionName)
      return i;
  }

  return -1;
}
