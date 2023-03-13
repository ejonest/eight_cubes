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
#include "main_widget.h"
#include <QtDebug>

//adjusted 3/13
#define OFFSET 6
#define CUBES 16
#define RADIUS 10

//added 3/13
MainWidget::MainWidget() : texture_cubes() {
    qDebug() << "Creating texture cubes...";
    for(int i = 0; i < CUBES; ++i) {
        texture_cubes.push_back(new TextureCube(0, 0));
    }
}

//adjusted 3/13
MainWidget::~MainWidget() {
    qDebug() << "Deleting texture cubes....";
    for(int i = 0; i < CUBES; ++i) {
        delete texture_cubes[i];
    }
}

void MainWidget::mousePressEvent(QMouseEvent* e) {
            // Save mouse press position
    mousePressPosition = QVector2D(e->position());
}

void MainWidget::mouseReleaseEvent(QMouseEvent* e) {
    // Mouse release position - mouse press position
    QVector2D diff = QVector2D(e->position()) - mousePressPosition;

            // Rotation axis is perpendicular
            //     to the mouse position difference vector
    QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();

            // Accelerate angular speed relative
            //     to the length of the mouse sweep
    qreal acc = diff.length() / 100.0;

            // Calculate new rotation axis as weighted sum
    rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();
    angularSpeed += acc;    // Increase angular speed
}

void MainWidget::timerEvent(QTimerEvent*) {
    // Decrease angular speed (friction)
//    angularSpeed *= 0.99;

    //for toggle speed of 8 cubes
    static bool fast = true;
    if (angularSpeed > 0 && angularSpeed <= 1) {
        fast = true;
    } else if (angularSpeed < 20 && angularSpeed >= 19) {
        fast = false;
    }
    if (fast == true) {
        angularSpeed *= 1.01;
    } else {
        angularSpeed *= .99;
    }
    if (angularSpeed < 0.01) {
        angularSpeed = 0.0;
    } else {
        rotation = QQuaternion::fromAxisAndAngle(rotationAxis, angularSpeed) * rotation;

        //adjusted 3/13 was orginally the ula, ura, etc. format
        for(int i = 0; i < CUBES; ++i) {
            texture_cubes[i]->set_rotation(rotation);
        }
//        ula.set_rotation(rotation);
//        ura.set_rotation(rotation);
//        bla.set_rotation(rotation);
//        bra.set_rotation(rotation);

//        ulb.set_rotation(rotation);
//        urb.set_rotation(rotation);
//        blb.set_rotation(rotation);
//        brb.set_rotation(rotation);


    update();
    }

//    angularSpeed *= 0.99;
    qDebug() << "angularSpeed = " << angularSpeed << (fast ? "Speeding up" : "Slowing down") ;
}

void MainWidget::initializeGL() {
    //adjusted 3/13 was orginally the ula, ura, etc. format
    //this method will form a circle not an octogon
    double x = 0;
    double y = 0;
    for(int i = 0; i < CUBES; ++i) {
        x = RADIUS * cos(i * 2.0 * M_PI / CUBES);
        y = RADIUS * sin(i * 2.0 * M_PI / CUBES);

        texture_cubes[i]->set_xy(x, y);
        texture_cubes[i]->initializeGL();
    }

//    ula.set_xy(-OFFSET / 2,  OFFSET);
//    ura.set_xy( OFFSET / 2,  OFFSET);
//    bla.set_xy(-OFFSET / 2, -OFFSET);
//    bra.set_xy( OFFSET / 2, -OFFSET);
//    ulb.set_xy(-OFFSET,  OFFSET / 2);
//    urb.set_xy( OFFSET,  OFFSET / 2);
//    blb.set_xy(-OFFSET, -OFFSET/ 2);
//    brb.set_xy( OFFSET, -OFFSET / 2);

//    ula.initializeGL();
//    ura.initializeGL();
//    bla.initializeGL();
//    bra.initializeGL();
//    ulb.initializeGL();
//    urb.initializeGL();
//    blb.initializeGL();
//    brb.initializeGL();

    timer.start(12, this);
}

void MainWidget::initShaders() {
    //adjusted 3/13 was orginally the ula, ura, etc. format
    for(int i = 0; i < CUBES; ++i) {
        texture_cubes[i]->initShaders();
    }
//    ula.initShaders();
//    ura.initShaders();
//    bla.initShaders();
//    bra.initShaders();
//    ulb.initShaders();
//    urb.initShaders();
//    blb.initShaders();
//    brb.initShaders();
}

void MainWidget::initTextures() {
    //adjusted 3/13 was orginally the ula, ura, etc. format
    for(int i = 0; i < CUBES; ++i) {
        texture_cubes[i]->initTextures();
    }
//    ula.initTextures();
//    ura.initTextures();
//    bla.initTextures();
//    bra.initTextures();
//    ulb.initTextures();
//    urb.initTextures();
//    blb.initTextures();
//    brb.initTextures();
}

void MainWidget::resizeGL(int w, int h) {
    //adjusted 3/13 was orginally the ula, ura, etc. format
    for(int i = 0; i < CUBES; ++i) {
        texture_cubes[i]->resizeGL(w, h);
    }
//    ula.resizeGL(w, h);
//    ura.resizeGL(w, h);
//    bla.resizeGL(w, h);
//    bra.resizeGL(w, h);
//    ulb.resizeGL(w, h);
//    urb.resizeGL(w, h);
//    blb.resizeGL(w, h);
//    brb.resizeGL(w, h);
}


void MainWidget::paintGL() {
//        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //adjusted 3/13 was orginally the ula, ura, etc. format
    for(int i = 0; i < CUBES; ++i) {
        texture_cubes[i]->paintGL();
    }
//    ula.paintGL();
//    ura.paintGL();
//    bla.paintGL();
//    bra.paintGL();
//    ulb.paintGL();
//    urb.paintGL();
//    blb.paintGL();
//    brb.paintGL();
}
