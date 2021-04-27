//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------


#ifndef MICO_AR_GLHELPERS_VISUALIZERGLWIDGET_H_
#define MICO_AR_GLHELPERS_VISUALIZERGLWIDGET_H_

#include <mico/ar/gl_helpers/Scene3d.h>

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QWheelEvent>
#include <QTimer>

#include <mutex>

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#ifndef M_PI
# define M_PI 3.14159265359 
#endif


namespace mico{
    namespace ar {
        /// QT widget to visualize in OpenGL the trajectory, map and any 3d information required
        /// @ingroup app_anita_gui
        class VisualizerGlWidget: public QOpenGLWidget, protected QOpenGLFunctions {
            Q_OBJECT
        public:
            explicit VisualizerGlWidget(QWidget *_parent = 0);
            ~VisualizerGlWidget();

            void addPoint(Scene3d::Point _p);
            void addLine(Scene3d::Point _p1, Scene3d::Point _p2);
            
            void updatePose(Eigen::Matrix4f _pose);

            void clearAll();

            void updateBackgroundImage(const cv::Mat& _image);

        public slots:
            void cleanup();

        protected:
            void initializeGL() override;
            void paintGL() override;
            void resizeGL(int width, int height) override;

            void keyReleaseEvent(QKeyEvent *event) override;
            void keyPressEvent(QKeyEvent *event) override;
            void mousePressEvent(QMouseEvent *event) override;
            void mouseReleaseEvent(QMouseEvent *event) override;
            void mouseMoveEvent(QMouseEvent *event) override;
            void wheelEvent(QWheelEvent *event) override;
        
            void drawBackground();

        private:
            Scene3d *scene_;
            
            QTimer *glTimer_;

            Eigen::Matrix4f pose_ = Eigen::Matrix4f::Identity();
            cv::Mat currentBg_;
        };
    }
}

#endif