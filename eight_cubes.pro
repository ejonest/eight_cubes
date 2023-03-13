QT       += core gui widgets
QT       += openglwidgets

TARGET = cube
TEMPLATE = app

SOURCES += main.cpp \
    texture_cube.cpp

SOURCES += \
    geometryengine.cpp \
    main_widget.cpp

HEADERS += \
    geometryengine.h \
    main_widget.h \
    texture_cube.h

RESOURCES += \
    shaders.qrc \
    textures.qrc

# install
target.path = $$[QT_INSTALL_EXAMPLES]/opengl/cube
INSTALLS += target

DISTFILES +=
