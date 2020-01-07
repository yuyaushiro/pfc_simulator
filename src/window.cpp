#include "window.h"


//------------------------------------------------------------------------------
// コンストラクタ
Window::Window(int width, int height, const char *title)
  : window(glfwCreateWindow(width, height, title, NULL, NULL))
{
  if (window==NULL)
  {
    // ウィンドウが作成できなかった
    std::cerr << "Can't create GLFW window." << std::endl;
  }

  // 現在のウィンドウを処理対象にする
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // 垂直同期のタイミングを待つ
  glfwSwapInterval(1);

  // このインスタンスの this ポインタを記録しておく
  glfwSetWindowUserPointer(window, this);

  // ウィンドウのサイズ変更時に呼び出す処理の登録
  glfwSetWindowSizeCallback(window, resize);

  // 開いたウィンドウの初期設定
  resize(window, width, height);
}


//------------------------------------------------------------------------------
// デストラクタ
Window::~Window()
{
  glfwDestroyWindow(window);
}


//------------------------------------------------------------------------------
// 描画ループの継続判定
Window::operator bool() const
{
  // ウィンドウを閉じる必要がなければ true を返す
  return !glfwWindowShouldClose(window);
}


//------------------------------------------------------------------------------
// ダブルバッファリング
void Window::swapBuffers() const
{
  // カラーバッファを入れ替える
  glfwSwapBuffers(window);
}


//------------------------------------------------------------------------------
// 縦横比を取り出す
const GLfloat* Window::getSize() const
{
  return size;
}


//------------------------------------------------------------------------------
// ウィンドウのサイズ変更時の処理
void Window::resize(GLFWwindow *const window, int width, int height)
{
  float y_range = 10.0;
  float x_range = y_range * width/height;

  // Retina Display 用の処理
  int fw, fh;
  glfwGetFramebufferSize(window, &fw, &fh);
  float scale = fw / width;

  glViewport(0, 0, static_cast<int>(width*scale), static_cast<int>(height*scale));

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-x_range/2, x_range/2,
          -y_range/2, y_range/2,
          -1.0, 1.0);

}
