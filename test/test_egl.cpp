#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <iostream>

int main() {
    // 1. Получаем дисплей
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY) {
        std::cerr << "Failed to get display" << std::endl;
        return 1;
    }

    // 2. Инициализируем EGL
    EGLint major, minor;
    if (!eglInitialize(display, &major, &minor)) {
        std::cerr << "Failed to initialize EGL" << std::endl;
        return 1;
    }
    std::cout << "EGL version: " << major << "." << minor << std::endl;

    // 3. Пробуем привязать OpenGL ES API
    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        std::cerr << "Failed to bind OpenGL ES API: 0x" << std::hex << eglGetError() << std::endl;
        return 1;
    }
    
    std::cout << "Successfully bound OpenGL ES API" << std::endl;
    
    eglTerminate(display);
    return 0;
}