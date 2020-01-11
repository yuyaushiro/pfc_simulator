#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <iostream>
#include <png.h>

#include "png_texture.h"


// コンストラクタ
//------------------------------------------------------------------------------
PngTexture::PngTexture()
{}


// コンストラクタ
//------------------------------------------------------------------------------
PngTexture::PngTexture(const std::string& fileName, GLuint textureId)
  : fileName_(fileName)
  , textureId_(textureId)
{
  setupTexture();
}


// デストラクタ
//------------------------------------------------------------------------------
PngTexture::~PngTexture()
{
  free(data_);
}


// テクスチャを設定する
//------------------------------------------------------------------------------
void PngTexture::setupTexture()
{
  png_structp sp = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  png_infop ip = png_create_info_struct(sp);
  FILE *fp = fopen(fileName_.c_str(), "rb");
  if (!fp)
  {
    perror(fileName_.c_str());
  }
  png_init_io(sp, fp);
  png_read_info(sp, ip);
  png_get_IHDR(sp, ip, (png_uint_32*)&width_, (png_uint_32*)&height_, &depth_, &colortype_, &interlacetype_, NULL, NULL);

  // メモリ領域確保
  int rb = png_get_rowbytes(sp, ip);
  data_ = new ubyte_t[height_ * rb];
  ubyte_t **recv = new ubyte_t*[height_];
  for (int i = 0; i < height_; i++)
    recv[i] = &data_[i * rb];
  png_read_image(sp, recv);
  png_read_end(sp, ip);
  png_destroy_read_struct(&sp, &ip, NULL);
  fclose(fp);
  delete[] recv;

  // テクスチャへの登録
  glBindTexture(GL_TEXTURE_2D, textureId_);
  glTexImage2D(GL_TEXTURE_2D, 1, GL_RGBA, width_, height_, 0, GL_RGBA, GL_UNSIGNED_BYTE, data_);

  // 画像が収縮されたときの処理
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // 画像が収縮されたときの処理
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  std::cout << height_ << width_ << std::endl;
}

// 描画
//------------------------------------------------------------------------------
void PngTexture::draw()
{
  const GLfloat vtx[] = {
    5.0f, 5.0f,
    -5.0f, 5.0f,
    -5.0f, -5.0f,
    5.0f, -5.0f
   };
  glVertexPointer(2, GL_FLOAT, 0, vtx);
  // 頂点ごとのテクスチャ座標を配列で準備
  const GLfloat texture_uv[] = {
    1, 0,
    0, 0,
    0, 1,
    1, 1,
  };
  glTexCoordPointer(2, GL_FLOAT, 0, texture_uv);

  // OpengGLにテクスチャによる描画を有効にすると指示
  glEnable(GL_TEXTURE_2D);

  glEnableClientState(GL_VERTEX_ARRAY);
  // 描画のときにテクスチャ座標配列も使うと指示
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);

  // 矩形を１つ描画
  glDrawArrays(GL_QUADS, 0, 4);

  // 描画が済んだら使った昨日を全て無効にする
  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_2D);
}
