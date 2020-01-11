#pragma once

#include <string>
#include <GLFW/glfw3.h>


typedef unsigned char ubyte_t;
class PngTexture
{
public:
  // コンストラクタ
  PngTexture();
  PngTexture(const std::string& fileName, GLuint textureId);

  // デストラクタ
  ~PngTexture();

  // テクスチャを設定する
  void setupTexture();

  void draw();

private:
  // PNG画像ファイルの名前
  std::string fileName_;

  // テクスチャのID
  GLuint textureId_;

  // 生データ
  ubyte_t *data_;

  // ピクセル数
  unsigned int width_, height_;

  int depth_, colortype_, interlacetype_;
};
