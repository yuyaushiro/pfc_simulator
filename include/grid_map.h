#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <GLFW/glfw3.h>

#include "pose.h"


class GridMap
{
public:
  /// コンストラクタ
  GridMap();

  /// コンストラクタ
  GridMap(const std::string& fileName, const Pose& minPose, const std::vector<double>& cellWidth);

  /// 障害物の中？
  bool insideObstacle(const Pose& pose) const;

  /// ピクセルインデックスに変換
  std::vector<int> toPixelIndex(const Pose& pose) const;

  /// データインデックスに変換
  int toDataIndex(const Pose& pose) const;

  // テクスチャの設定
  void setTexture();

  /// 描画
  void draw();

private:
  /// ファイル名
  std::string fileName_;

  /// マップ画像
  cv::Mat image_;

  /// マップ画像（グレースケール）
  cv::Mat grayImage_;

  /// 画像サイズ
  std::vector<int> pixelNum_;

  /// 1セルあたりの幅 [m/cell, m/cell, rad/cell]
  std::vector<double> cellWidth_;

  /// 最小の姿勢
  Pose minPose_;

  /// 障害物閾値
  int obstacleThreshold_;

  /// テクスチャID
  GLuint textureId_;
};


