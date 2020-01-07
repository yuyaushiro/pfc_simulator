#include "cmd_vel.h"


/// コンストラクタ
//------------------------------------------------------------------------------
CmdVel::CmdVel()
  : nu(0.1)
  , omega(0.5)
{}


/// コンストラクタ
//------------------------------------------------------------------------------
CmdVel::CmdVel(double nu, double omega, std::string name)
  : nu(nu)
  , omega(omega)
  , name(name)
{}


// ストリームオーバーロード
// ------------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& os, const CmdVel& cmdVel)
{
  os << cmdVel.nu << ", " << cmdVel.omega;
  return os;
}
