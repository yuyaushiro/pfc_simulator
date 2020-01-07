#pragma once

#include <iostream>


// 行動
class CmdVel
{
public:
  /// コンストラクタ
  CmdVel();
  CmdVel(double nu, double omega, const std::string name);

  /// デストラクタ
  ~CmdVel() = default;

  /// ストリームオーバーロード
  friend std::ostream& operator<<(std::ostream& os, const CmdVel& cmdVel);

  /// 直進速度
  double nu;

  /// 回転速度
  double omega;

  /// コマンド名
  std::string name;
};
