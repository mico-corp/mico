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


#ifndef MICO_AR_GL_HELPERS_SCENE3D_H_
#define MICO_AR_GL_HELPERS_SCENE3D_H_


#include <string>
#include <vector>
#include <mico/ar/gl_helpers/Mesh.h>
#include <memory>

#include <Eigen/Eigen>
#include <map>

#if defined(WIN32)
#   include <GL/freeglut.h> // GLUT, include glu.h and gl.h
#elif defined(linux)
#   include <GL/glut.h> // GLUT, include glu.h and gl.h
#endif

namespace mico{
    namespace ar {
        /// Handler of 3d scene to be display using OpenGL
        /// @ingroup aerox_gl
        class Scene3d{
        public:
            struct Point {
                float x,y,z;
            };

        public:

            static Scene3d* get() {
                if (currentInstance_ == nullptr) {
                    currentInstance_ = new Scene3d();
                    currentInstance_->init();
                }
                return currentInstance_;
            }


            void stop();

            int addAxis(const Eigen::Matrix4f &_pose);
            void updateAxis(int _id, const Eigen::Matrix4f &_pose);
            void deleteAxis(int _id);

            void addMesh(std::shared_ptr<Mesh> _mesh);

            void addLine(Point _p1, Point _p2);

            void addPointCloud(std::vector<Point> _cloud);  // 666 can make it better

            void addMap(std::vector<Point> _centers, float _side);

            void clearPointClouds() {clouds_.clear();} ;

            void moveCamera(Eigen::Matrix4f _pose);

            void attachKeyboardfunction(std::function<void(unsigned char, int, int)> _fn);

            void clearGl();
            void drawAll();
            void swapBuffers();

            void resizeGL(int _width, int _height);

            void setCameraFov(float _fov);

            void flip() { flip_ = !flip_; };

        private:
            Scene3d() {};
            bool init();

            void setMaterial(   std::vector<float> _ambient = {0.7f, 0.7f, 0.7f, 1.0f}, 
                                std::vector<float> _diffuse = {1.0f, 1.0f, 1.0f, 1.0f}, 
                                std::vector<float> _specular = {1.0f, 1.0f, 1.0f, 1.0f}, 
                                float _shininess = 5.0f);

            void drawSquare     (const std::vector<Point> &_vertices, const Point &_normal);
            void drawCube       (const std::vector<Point> &_vertices);
            void drawCube       (std::vector<std::vector<Point>> _faces);

            void drawAxis(const Eigen::Matrix4f _pose);

            void drawOccupancyMap();

            bool getHeatMapColor(float value, float *red, float *green, float *blue);
        private:
            static Scene3d* currentInstance_;
            static GLfloat currentAspect_;
            
        private:
            /* Global variables */
            std::string title_ = "3D Shapes with animation";
            int refreshMills_ = 15;       // refresh interval in milliseconds [NEW]

            std::vector<std::shared_ptr<Mesh> > meshes_;
            std::vector<std::pair<Point, Point>> lines_;
            std::vector<std::vector<Point>> clouds_;
            std::vector<Point> centers_;
            std::map<int, Eigen::Matrix4f> axis_;
            int counterAxis_ = 0;
            float side_ = 0.1;

            Eigen::Matrix4f cameraPose_;

            static std::vector<std::function<void(unsigned char, int, int)>> keyboardCallbacks_;

            bool useGlut_ = false;

            float fov_ = 45.0f;

            bool flip_ = false;

        private:

        };
    }
}

#endif