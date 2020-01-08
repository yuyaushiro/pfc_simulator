#include <cmath>
#include <fstream>
#include <sstream>

#include "state_space.h"


// コンストラクタ
//------------------------------------------------------------------------------
StateSpace::StateSpace()
{}


// コンストラクタ
//------------------------------------------------------------------------------
StateSpace::StateSpace(const std::string& fileName, const std::vector<int>& cellNum,
                       const std::vector<double>& cellWidth, const Pose& minPose,
                       const std::vector<CmdVel>& cmdVels)
  : valueFileName_("../value/" + fileName + ".value")
  , transFileName_("../trans/" + fileName + ".trans")
  , cellNum_(cellNum)
  , cellWidth_(cellWidth)
  , minPose_(minPose)
  , cmdVels_(cmdVels)
{
  // ファイルを読み込む
  loadValueFile();
  loadTransFile();
}


// 状態価値を取得
double StateSpace::getValue(const Pose& pose) const
{
  double value = -( state_[toStateIndex(pose)].getValue() * 0.001 );
  if (value == 0.0)
    value = -1e-10;
  return value;
}


// 行動価値を取得
//------------------------------------------------------------------------------
double StateSpace::getActionValue(const Pose& pose, const CmdVel& cmdVel) const
{
  unsigned long stateIndex = toStateIndex(pose);
  int actionIndex = getActionIndex(cmdVel.name);
  std::vector<Transition> transitions = state_[stateIndex].getTransitions(actionIndex);
  double posteriorValue = 0;
  double reward = 0;
  for (int i = 0; i < transitions.size(); i++)
  {
    double probability = transitions[i].probability;
    reward += transitions[i].reward * probability;
    posteriorValue += state_[transitions[i].posteriorState].getValue() * probability;
  }

  return -( (posteriorValue + reward) * 0.001 );
}


// 状態遷移の報酬を取得
//------------------------------------------------------------------------------
double StateSpace::getReward(const Pose& pose, const CmdVel& cmdVel) const
{
  unsigned long stateIndex = toStateIndex(pose);
  int actionIndex = getActionIndex(cmdVel.name);

  std::vector<Transition> transitions = state_[stateIndex].getTransitions(actionIndex);
  double reward = 0;
  for (int i = 0; i < transitions.size(); i++)
  {
    double probability = transitions[i].probability;
    reward += transitions[i].reward * probability;
  }

  return -( reward * 0.001 );
}


// 状態遷移後の価値を取得
//------------------------------------------------------------------------------
double StateSpace::getPosteriorValue(const Pose& pose, const CmdVel& cmdVel) const
{
  unsigned long stateIndex = toStateIndex(pose);
  int actionIndex = getActionIndex(cmdVel.name);

  std::vector<Transition> transitions = state_[stateIndex].getTransitions(actionIndex);
  double posteriorValue = 0;
  for (int i = 0; i < transitions.size(); i++)
  {
    double probability = transitions[i].probability;
    posteriorValue += state_[transitions[i].posteriorState].getValue() * probability;
  }

  return -( posteriorValue * 0.001 );
}


// インデックスへの変換
//------------------------------------------------------------------------------
unsigned long StateSpace::toStateIndex(const Pose& pose) const
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
void StateSpace::loadValueFile()
{
  // サイズ変更
  size_t stateNum = cellNum_[0] * cellNum_[1] * cellNum_[2];
  state_.resize(stateNum);

  // 行動のリスト
  std::vector<Action> actions(3);

  // ファイル読み込み
  std::ifstream ifs(valueFileName_);
  std::string line;
  while (getline(ifs, line))
  {
    std::vector<std::string> words = splitOneLine(line, ' ');
    size_t index = std::stoul(words[0]);
    double value = std::stod(words[1]);
    state_[index] = State(value, actions);
  }

  ifs.close();
}


// 状態遷移ファイルを読み込む
//------------------------------------------------------------------------------
void StateSpace::loadTransFile()
{
  std::ifstream ifs(transFileName_);
  std::string line;

  // ヘッダー
  while (getline(ifs, line))
  {
    if (line == "%%state transitions%%")
    {
      break;
    }
  }

  // 状態遷移
  while (getline(ifs, line))
  {
    if (parseStateTrans(line) == false)
      break;
  }
}


// 状態遷移部分を分割
//------------------------------------------------------------------------------
bool StateSpace::parseStateTrans(std::string& line)
{
  std::vector<std::string> words = splitOneLine(line, ' ');
  // 空白行
  if (words.size() < 1)
    return true;

  // ファイルの状態遷移部分終了
  if (words[0].at(0) == '%')
    return false;

  static int stateIndex = -1;
  static int actionIndex = -1;
  if (words[0] == "state")
  {
    stateIndex = atoi(words[1].c_str());
    actionIndex = getActionIndex(words[3]);

    if (stateIndex < 0)
      return false;
  }
  else if (words[0][0] == '\t')
  {
    // 確率
    double probability = atof(words[3].c_str());
    // 後の状態
    unsigned long posteriorStateIndex = atoi(words[1].c_str());
    // 報酬
    unsigned long reward = atof(words[5].c_str());

    Transition transition = {probability, posteriorStateIndex, reward};
    state_[stateIndex].addTransition(actionIndex, transition);
  }

  return true;
}


// 1行を分割
//------------------------------------------------------------------------------
std::vector<std::string> StateSpace::splitOneLine(std::string& line, char delimiter)
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
int StateSpace::getActionIndex(const std::string& actionName) const
{
  for (int i = 0; i < cmdVels_.size(); i++)
  {
    if (cmdVels_[i].name == actionName)
      return i;
  }

  return -1;
}
