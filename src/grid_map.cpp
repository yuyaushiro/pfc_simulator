#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "grid_map.h"


// コンストラクタ
//------------------------------------------------------------------------------
GridMap::GridMap()
{}


// コンストラクタ
//------------------------------------------------------------------------------
GridMap::GridMap(const std::string& fileName, const std::vector<double>& cellWidth, const Pose& minPose)
  : fileName_("../map/" + fileName + ".png")
  , cellWidth_(cellWidth)
  , minPose_(minPose)
  , obstacleThreshold_(250)
{
  cv::Mat image = cv::imread(fileName_);
  cv::flip(image, image, 0);
  cv::cvtColor(image, grayImage_, cv::COLOR_BGR2GRAY);
  cv::cvtColor(image, image_, cv::COLOR_BGRA2RGBA);

  pixelNum_.push_back(image.cols);
  pixelNum_.push_back(image.rows);

  setTexture();
}


// 障害物の中？
//------------------------------------------------------------------------------
bool GridMap::insideObstacle(const Pose& pose) const
{
  int dataIndex = toDataIndex(pose);
  int pixelValue = grayImage_.data[dataIndex];
  if (pixelValue < obstacleThreshold_)
    return true;
  return false;
}


// ピクセルインデックスに変換
//------------------------------------------------------------------------------
int GridMap::toDataIndex(const Pose& pose) const
{
  std::vector<int> index(toPixelIndex(pose));
  return index[1]*pixelNum_[0] + index[0];
}


// データインデックスに変換
//------------------------------------------------------------------------------
std::vector<int> GridMap::toPixelIndex(const Pose& pose) const
{
  std::vector<int> index(2);
  Pose poseInState = pose - minPose_;
  index[0] = static_cast<int>(floor(poseInState.x / cellWidth_[0]));
  index[1] = static_cast<int>(floor(poseInState.y / cellWidth_[1]));

  // 正規化
  for (int i = 0; i < 2; i++)
  {
    if (index[i] < 0)
      index[i] = 0;
    else if (index[i] >= pixelNum_[i])
      index[i] = pixelNum_[i] - 1;
  }

  return index;
}


// テクスチャの設定
//------------------------------------------------------------------------------
void GridMap::setTexture()
{
  glGenTextures(1, &textureId_);

  // テクスチャへの登録
  glBindTexture(GL_TEXTURE_2D, textureId_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, pixelNum_[0], pixelNum_[1], 0, GL_RGBA, GL_UNSIGNED_BYTE,
               image_.data);

  // 画像が収縮されたときの処理
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  // 画像が収縮されたときの処理
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

  glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_FALSE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
}


// 描画
//------------------------------------------------------------------------------
void GridMap::draw(GLfloat range)
{
  glColor3f(1.0f, 1.0f, 1.0f);
  const GLfloat vtx[] = {
    -range, -range,
    range, -range,
    range, range,
    -range, range
   };
  glVertexPointer(2, GL_FLOAT, 0, vtx);
  // 頂点ごとのテクスチャ座標を配列で準備
  const GLfloat texture_uv[] = {
    0, 0,
    1, 0,
    1, 1,
    0, 1,
  };
  glTexCoordPointer(2, GL_FLOAT, 0, texture_uv);

  // OpengGLにテクスチャによる描画を有効にすると指示
  glEnable(GL_TEXTURE_2D);
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);

  // 矩形を１つ描画
  glDrawArrays(GL_QUADS, 0, 4);

  // 描画が済んだら使った昨日を全て無効にする
  glDisable(GL_TEXTURE_2D);
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}
