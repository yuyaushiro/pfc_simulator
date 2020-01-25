#pragma once

#include <string>
#include <vector>
#include <fstream>

#include "grid_map.h"
#include "goal.h"
#include "particle.h"


class Saver
{
public:
  /// コンストラクタ
  Saver();

  /// コンストラクタ
  Saver(const std::string& saveFileName, const GridMap& gridMap,
        const Goal& goal);

  /// 1ステップセーブ
  void saveOneStep(const std::vector<Particle>& particles);

  /// 1試行セーブ
  void saveOneTrial(bool goal);

  /// 状況の書き込み
  void writeToFile();

  /// 一行書き込み
  void writeLine(std::string oneLine);

  /// 経過ステップ
  long elapsedTime_;

  /// パーティクルが障害物内にいた時間
  long particlesInObstacle_;

  /// 試行回数
  long trialNum_;

private:
  /// 保存ファイル名
  std::string saveFileName_;

  /// 出力ストリーム
  std::ofstream ofs_;

  /// マップ
  GridMap gridMap_;
};
