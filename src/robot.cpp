#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "robot.h"
#include "circle.h"
#include "line.h"

// コンストラクタ
//------------------------------------------------------------------------------
Robot::Robot()
{}


// コンストラクタ
//------------------------------------------------------------------------------
Robot::Robot(const Pose& initialPose, const Goal& goal, const Mcl& mcl, const Pfc& pfc,
             int seedValue, const std::vector<double> motionStd)
  : pose_(initialPose)
  , goal_(goal)
  , mcl_(mcl)
  , pfc_(pfc)
  , mt_(seedValue)
  , motionStd_(motionStd)
  , trajectory_{initialPose}
  , radius_(0.15)
{}


// 状態を遷移させる
//------------------------------------------------------------------------------
Pose Robot::transitionStateWithNoise(const CmdVel& cmdVel, double dt, const Pose& pose)
{
  //乱数生成クラス
  std::normal_distribution<> nuDist(0, motionStd_[0]);
  std::normal_distribution<> omegaDist(0, motionStd_[1]);

  // 乱数生成
  std::vector<double> ns{nuDist(mt_), 0.0, 0.0, omegaDist(mt_)};

  double pnu = cmdVel.nu + ns[0]*sqrt(abs(cmdVel.nu)/dt) + ns[1]*sqrt(abs(cmdVel.omega)/dt);
  double pomega = cmdVel.omega + ns[2]*sqrt(abs(cmdVel.nu)/dt) + ns[3]*sqrt(abs(cmdVel.omega)/dt);

  // ノイズを含んだ行動
  CmdVel noisyCmdVel(pnu, pomega, "");
  pose_ = Robot::transitionState(noisyCmdVel, dt, pose_);
  return pose_;
}


// 状態を遷移させる
//------------------------------------------------------------------------------
Pose Robot::transitionState(const CmdVel& cmdVel, double dt, const Pose& pose)
{
  double x;
  double y;
  double theta;
  if (fabs(cmdVel.omega) < 1e-10)
  {
    x = pose.x + cmdVel.nu*cos(pose.theta)*dt;
    y = pose.y + cmdVel.nu*sin(pose.theta)*dt;
    theta = pose.theta + cmdVel.omega*dt;
  }
  else
  {
    x = pose.x + cmdVel.nu/cmdVel.omega*(sin(pose.theta + cmdVel.omega*dt) - sin(pose.theta));
    y = pose.y + cmdVel.nu/cmdVel.omega*(-cos(pose.theta + cmdVel.omega*dt) + cos(pose.theta));
    theta = pose.theta + cmdVel.omega*dt;
  }
  Pose afterPose(x, y, theta);
  return afterPose;
}


// 1ステップ進める
//------------------------------------------------------------------------------
void Robot::oneStep(double dt)
{
  // 意思決定
  CmdVel cmdVel = pfc_.decisionMaking(mcl_.particles_, dt);
  // CmdVel cmdVel = pfc_.decisionMaking(mcl_.averagePose_, dt);
  // CmdVel cmdVel = pfc_.decisionMaking(pose_, dt);

  // 状態遷移
  // pose_ = Robot::transitionState(cmdVel, dt, pose_);
  pose_ = Robot::transitionStateWithNoise(cmdVel, dt, pose_);

  // 推定器のモーションアップデート
  mcl_.updateWithMotion(cmdVel, dt);

  // ロボットがゴールしていなければ
  // if (!goal_.inside(pose_))
    // 推定器のゴール観測によるアップデート
    mcl_.updateWithGoalObservation(goal_);

  // 推定器のリサンプリング
  mcl_.resampling();
  // 平均姿勢を計算
  mcl_.calcAveragePose();

  // 軌跡の追加
  trajectory_.push_back(pose_);
}


// 姿勢を取得
//------------------------------------------------------------------------------
Pose Robot::getPose()
{
  return pose_;
}


// 姿勢を取得
//------------------------------------------------------------------------------
std::vector<Particle>& Robot::getParticles()
{
  return mcl_.particles_;
}


// リスタート
//------------------------------------------------------------------------------
void Robot::restart(const Pose& initialPose, const std::vector<double> initPoseStd)
{
  // ロボットの初期化
  std::normal_distribution<double> xDist(initialPose.x, initPoseStd[0]);
  std::normal_distribution<double> yDist(initialPose.y, initPoseStd[1]);
  std::normal_distribution<double> thetaDist(initialPose.theta, initPoseStd[2]);
  Pose pose(xDist(mt_), yDist(mt_), thetaDist(mt_));
  pose_ = pose;
  trajectory_.resize(1);
  trajectory_[0] = pose;

  // 推定器の初期化
  mcl_.init(initialPose, initPoseStd);
}


// 描画
//------------------------------------------------------------------------------
void Robot::draw()
{
  // パーティクルの描画
  mcl_.draw();

  // ロボットの描画
  Circle body(0, 0, radius_, 3);
  Point point1(0, 0);
  Line rotation(point1, 0, radius_*1.5, 3);
  glPushMatrix();
  glTranslated(pose_.x, pose_.y, 0.0);
  glRotated(pose_.theta*180/M_PI, 0, 0, 1);
  body.draw();
  rotation.draw();
  glPopMatrix();

  // 軌跡の描画
  std::vector<GLfloat> vtx;
  int vtxSize = trajectory_.size()*2;
  vtx.resize(vtxSize);
  int j = 0;
  for (int i = 0; i < vtxSize; i = i + 2)
  {
    vtx[i] = static_cast<GLfloat>(trajectory_[j].x);
    vtx[i+1] = static_cast<GLfloat>(trajectory_[j].y);
    j++;
  }
  glVertexPointer(2, GL_FLOAT, 0, &vtx[0]);
  glEnableClientState(GL_VERTEX_ARRAY);
  glColor3f(0.0f, 0.0f, 0.0f);
  glLineWidth(1);
  glDrawArrays(GL_LINE_STRIP, 0, static_cast<int>(vtxSize/2));
  glDisableClientState(GL_VERTEX_ARRAY);
}
