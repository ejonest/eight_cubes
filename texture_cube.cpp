/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtCore module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/


#include <QMouseEvent>

#include <cmath>

#include "texture_cube.h"
#include "main_widget.h"


TextureCube::TextureCube(int x, int y)
: xoffset(x), yoffset(y) {
    qDebug("TextureCube created\n");
}

TextureCube::~TextureCube() {
            // Make sure the context is current when deleting the texture
            // and the buffers.
    makeCurrent();
    delete texture;
    delete geometries;
    doneCurrent();
}

void TextureCube::mousePressEvent(QMouseEvent*) { }

void TextureCube::mouseReleaseEvent(QMouseEvent*) {}

void TextureCube::timerEvent(QTimerEvent *) { }

void TextureCube::set_xy(int x, int y) {
    xoffset = x;
    yoffset = y;
}

void TextureCube::set_rotation(QQuaternion& rotation_val) { rotation = rotation_val; }

void TextureCube::initOpenGL() { initializeOpenGLFunctions(); }

void TextureCube::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0, 0, 0, 1);

    initShaders();
    initTextures();

    glEnable(GL_DEPTH_TEST);   // Enable depth buffer
    glEnable(GL_CULL_FACE);    // Enable back face culling

    geometries = new GeometryEngine;
}

void TextureCube::initShaders() {
                 // Compile vertex shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Vertex,
                                         ":/vshader.glsl")) { close(); }
                 // Compile fragment shader
    if (!program.addShaderFromSourceFile(QOpenGLShader::Fragment,
                                         ":/fshader.glsl")) { close(); }

    if (!program.link()) { close(); }   // Link shader pipeline

    if (!program.bind()) { close(); }   // Bind shader pipeline for use
}

void TextureCube::initTextures() {
            // Load cube.png image
    texture = new QOpenGLTexture(QImage(":/cube.png").mirrored());

            // Set nearest filtering mode for texture minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);

            // Set bilinear filtering mode for texture magnification
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

            // Wrap texture coordinates by repeating
            // f.ex. texture coordinate (1.1, 1.2) is same as (0.1, 0.2)
    texture->setWrapMode(QOpenGLTexture::Repeat);
}

void TextureCube::resizeGL(int w, int h) {
    qreal aspect = qreal(w) / qreal(h ? h : 1);  // Calculate aspect ratio

    //    const qreal zNear = 3.0, zFar = 7.0, fov = 45.0;
    //adjusted 3/13
    //the zNear is how close you see, zFar how far, and fov is feild of vison
    //sort of like the degree view on the backup camera of a car
    const qreal zNear = 1.0, zFar = 17.0, fov = 80.0;


    projection.setToIdentity();  // Reset projection

            // Set perspective projection
    projection.perspective(fov, aspect, zNear, zFar);
}

void TextureCube::paintGL() {
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    texture->bind();

            // Calculate model view transformation
    QMatrix4x4 matrix;
    //adjusted 3/13
    //adjusted the offset to make the camera have all cubes in frame
    matrix.translate(xoffset, yoffset, -15.0);
    matrix.rotate(rotation);

            // Set modelview-projection matrix
    program.setUniformValue("mvp_matrix", projection * matrix);

            // Use texture unit 0 which contains cube.png
    program.setUniformValue("texture", 0);

            // Draw cube geometry
    geometries->drawCubeGeometry(&program);
}
