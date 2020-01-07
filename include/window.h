#pragma once

#include <iostream>
#include <GLFW/glfw3.h>


/// ウィンドウ
class Window
{
public:
  /// コンストラクタ
  Window(int width=800, int height=800, const char *title="Hello");

  /// デストラクタ
  virtual ~Window();

  /// 描画ループの継続判定
  explicit operator bool() const;

  /// ダブルバッファリング
  void swapBuffers() const;

  /// ウィンドウサイズを取り出す
  const GLfloat* getSize() const;

  /// ウィンドウのサイズ変更時の処理
  static void resize(GLFWwindow *const window, int width, int height);

private:
  /// ウィンドウのハンドル
  GLFWwindow *const window;

  /// 縦横比
  GLfloat size[2];
};
