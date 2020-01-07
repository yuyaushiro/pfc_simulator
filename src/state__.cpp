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
State::State(const std::string& fileName, const std::vector<int>& cellNum,
             const std::vector<double>& cellWidth, const Pose& minPose,
             const std::vector<CmdVel>& cmdVels)
  : valueFileName_("../value/" + fileName + ".value")
  , transFileName_("../stp/" + fileName + ".stp")
  , cellNum_(cellNum)
  , cellWidth_(cellWidth)
  , minPose_(minPose)
  , cmdVels_(cmdVels)
{
  // ファイルを読み込む
  loadValueFile();
  loadTransFile();
}


// セルインデックスに変換
//------------------------------------------------------------------------------
std::vector<int> State::toCellIndex(const Pose& pose) const
{
  std::vector<int> cellIndex(3);
  Pose poseInState = pose - minPose_;
  cellIndex[0] = static_cast<int>(floor(poseInState.x / cellWidth_[0]));
  cellIndex[1] = static_cast<int>(floor(poseInState.y / cellWidth_[1]));
  cellIndex[2] = static_cast<int>(floor(poseInState.theta / cellWidth_[2]));

  // 正規化
  for (int i = 0; i < 2; i++)
  {
    if (cellIndex[i] < 0)
      cellIndex[i] = 0;
    else if (cellIndex[i] >= cellNum_[i])
      cellIndex[i] = cellNum_[i] - 1;
  }
  cellIndex[2] = (cellIndex[2] + cellNum_[2]*1000) % cellNum_[2];

  return cellIndex;
}


// 状態インデックスに変換
//------------------------------------------------------------------------------
unsigned long State::toStateIndex(std::vector<int>& cellIndex) const
{
  unsigned long stateIndex =
        cellIndex[0]*cellNum_[1]*cellNum_[2] + cellIndex[1]*cellNum_[2] + cellIndex[2];
  return stateIndex;
}


// 価値関数ファイルを読み込む
//------------------------------------------------------------------------------
void State::loadValueFile()
{
  // サイズ変更
  size_t stateNum = cellNum_[0] * cellNum_[1] * cellNum_[2];
  value_.resize(stateNum);

  // ファイル読み込み
  std::ifstream ifs(valueFileName_);
  std::string line;
  while (getline(ifs, line))
  {
    std::vector<std::string> words = splitOneLine(line, ' ');
    size_t index = std::stoul(words[0]);
    double value = std::stod(words[1]);
    value_[index] = value;
  }

  ifs.close();
}


// 状態遷移ファイルを読み込む
//------------------------------------------------------------------------------
void State::loadValueFile()
{
  // サイズ変更
  transProb_.resize(cmdVels_.size());
  for (int i = 0; i < transProb_.size(); i++)
  {
    transProb_[i].resize(cellNum_[2]);
  }

  // ファイル読み込み
  std::ifstream ifs(transFileName_);
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
  }

  ifs.close();
}


// 1行を分割
//------------------------------------------------------------------------------
std::vector<std::string> State::splitOneLine(std::string& line, char delimiter)
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
