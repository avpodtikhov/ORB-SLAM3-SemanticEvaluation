#include <EGL/egl.h>
#include <iostream>

int main() {
    EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (display == EGL_NO_DISPLAY) {
        std::cout << "Failed to get EGL display" << std::endl;
        return 1;
    }
    
    EGLint major, minor;
    if (!eglInitialize(display, &major, &minor)) {
        std::cout << "Failed to initialize EGL" << std::endl;
        return 1;
    }
    
    std::cout << "EGL version: " << major << "." << minor << std::endl;
    return 0;
}
