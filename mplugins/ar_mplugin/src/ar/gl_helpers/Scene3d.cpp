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


#include <mico/ar/gl_helpers/Scene3d.h>

namespace mico{
    namespace ar {
        Scene3d* Scene3d::currentInstance_ = nullptr;
        GLfloat Scene3d::currentAspect_=1;
        std::vector<std::function<void(unsigned char, int, int)>> Scene3d::keyboardCallbacks_ = {};

        bool Scene3d::init(){

            if (currentInstance_ != nullptr)
                return false;

            glClearColor(0.3f, 0.3f, 0.3f, 3.0f);              // Set background color to black and opaque
            glClearDepth(1.0f);                                // Set background depth to farthest
            glDepthFunc(GL_LEQUAL);                            // Set the type of depth-test
            glEnable(GL_DEPTH_TEST);
            glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST); // Nice perspective corrections
            
            glEnable(GL_LIGHTING);
            GLfloat light_position1[] = { -3.0, 0.0, 5.0, 0.0 };
            glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
            glEnable(GL_LIGHT0);

            GLfloat light_position2[] = { 3.0, 0.0, 5.0, 0.0 };
            glLightfv(GL_LIGHT1, GL_POSITION, light_position2);
            glEnable(GL_LIGHT1);

            glShadeModel (GL_SMOOTH);

            return true;
        }

        void Scene3d::stop(){
            if(useGlut_){
                if(currentInstance_ == nullptr)
                    return;

                glutMainLoop();                   // Enter the infinite event-processing loop
            }
        }


        int Scene3d::addAxis(const Eigen::Matrix4f &_pose){
            counterAxis_++;
            axis_[counterAxis_] = _pose;
            return counterAxis_; 
        }

        void Scene3d::updateAxis(int _id, const Eigen::Matrix4f &_pose){
            if(axis_.find(_id) != axis_.end())
                axis_[_id] = _pose;
        }

        void Scene3d::deleteAxis(int _id){
            if(axis_.find(_id) != axis_.end())
                axis_.erase(_id);   
        }

        void Scene3d::addMesh(std::shared_ptr<Mesh>  _mesh){
            meshes_.push_back(_mesh);
        }

        void Scene3d::addLine(Point _p1, Point _p2){
            lines_.push_back({_p1, _p2});
        }

        void Scene3d::addPointCloud(std::vector<Point> _cloud){
            clouds_.push_back(_cloud);
        }

        void Scene3d::addMap(std::vector<Point> _centers, float _side){
            centers_ = _centers;
            side_ = _side;
        }

        void Scene3d::moveCamera(Eigen::Matrix4f _pose){
            cameraPose_ = _pose;
        }

        void Scene3d::attachKeyboardfunction(std::function<void(unsigned char, int, int)> _fn){
            keyboardCallbacks_.push_back(_fn);
        }

        void Scene3d::clearGl() {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
        }
        
        void Scene3d::drawAll(){

            glEnable(GL_LIGHTING);
            GLfloat light_position1[] = { -3.0, 0.0, 5.0, 0.0 };
            glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
            glEnable(GL_LIGHT0);

            // Place camera
            glMatrixMode(GL_PROJECTION); // To operate on the Projection matrix
            glLoadIdentity();            // Reset
            gluPerspective(fov_, currentAspect_, 0.1f, 100.0f);
            gluLookAt(	cameraPose_(0,3),
                        cameraPose_(1,3),
                        cameraPose_(2,3),
                        cameraPose_(0,3) + cameraPose_(0,2)*1.0,
                        cameraPose_(1,3) + cameraPose_(1,2)*1.0,
                        cameraPose_(2,3) + cameraPose_(2,2)*1.0,
                        cameraPose_(0,1),
                        cameraPose_(1,1),
                        cameraPose_(2,1));

            // Draw meshes
            setMaterial({0.7f, 0.7f, 0.7f, 1.0f});
            for(auto &mesh: meshes_){
                mesh->displayMesh();
            }

            // Draw lines
            glLineWidth(3.0);
            for(auto &line: lines_){
                setMaterial({0.3f, 1.0f, 0.3f, 1.0f});
                glEnable(GL_COLOR_MATERIAL);
                glBegin(GL_LINES);
                    glVertex3f(line.first.x, line.first.y, line.first.z);
                    glVertex3f(line.second.x, line.second.y, line.second.z);
                glEnd();
            glEnable(GL_LIGHTING);
            }
            glLineWidth(1.0f);
            setMaterial();

            // Draw point clouds
            glBegin (GL_POINTS);
            for(auto &cloud: clouds_){
                for (auto &p:cloud) {
                    glVertex3f (p.x, p.y, p.z);
                }
            }
            glEnd ();

            // Draw coordinates
            for(auto &[id, pose]: axis_){
                drawAxis(pose);
            }

        }

        void Scene3d::swapBuffers() {
            if (useGlut_) {
                glutSwapBuffers(); // Swap the front and back frame buffers (double buffering)
            }
        }


        void Scene3d::resizeGL(int _width, int _height){
            currentAspect_ = (float)_width / (float)_height;
            glViewport(_width, _height, 0, 0);
        }


        void Scene3d::setCameraFov(float _fov) {
            fov_ = _fov;
        }


        void Scene3d::setMaterial(std::vector<float> _ambient, std::vector<float> _diffuse, std::vector<float> _specular, float _shininess){
            glColor4f(_ambient[0], _ambient[1], _ambient[2], _ambient[3]);
            glMaterialfv(GL_FRONT, GL_AMBIENT, _ambient.data());
            glMaterialfv(GL_FRONT, GL_DIFFUSE, _diffuse.data());
            glMaterialfv(GL_FRONT, GL_SPECULAR, _specular.data());
            glMaterialf(GL_FRONT, GL_SHININESS, _shininess);
        }

        void Scene3d::drawSquare(const std::vector<Point> &_vertices, const Point &_normal){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix

            float r,g,b;
            float midZ = (_vertices[0].z+_vertices[1].z+_vertices[2].z+_vertices[3].z)/4;
            getHeatMapColor(midZ, &r, &g, &b);
            setMaterial({r, g, b, 1.0f});
            glBegin(GL_POLYGON);
                glVertex3f(_vertices[0].x, _vertices[0].y, _vertices[0].z);
                glVertex3f(_vertices[1].x, _vertices[1].y, _vertices[1].z);
                glVertex3f(_vertices[2].x, _vertices[2].y, _vertices[2].z);
                glVertex3f(_vertices[3].x, _vertices[3].y, _vertices[3].z);
                glNormal3f(_normal.x, _normal.y, _normal.z);
            glEnd();

            setMaterial({0.0f, 0.0f, 0.0f, 1.0f});
            glLineWidth(3.0);
            glBegin(GL_LINES);
                glVertex3f(_vertices[0].x, _vertices[0].y, _vertices[0].z);
                glVertex3f(_vertices[1].x, _vertices[1].y, _vertices[1].z);
                glVertex3f(_vertices[2].x, _vertices[2].y, _vertices[2].z);
                glVertex3f(_vertices[3].x, _vertices[3].y, _vertices[3].z);
            glEnd();
            glLineWidth(1.0);
        }

        void Scene3d::drawCube(const std::vector<Point> &_v){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix
            drawSquare({_v[3], _v[2], _v[1], _v[0]}, {0,0,1});
            drawSquare({_v[4], _v[5], _v[6], _v[7]}, {0,0,-1});
            drawSquare({_v[4], _v[7], _v[3], _v[0]}, {0,-1,0});
            drawSquare({_v[7], _v[6], _v[2], _v[3]}, {1,0,0});
            drawSquare({_v[6], _v[5], _v[1], _v[2]}, {0,1,0});
            drawSquare({_v[0], _v[1], _v[5], _v[4]}, {-1,0,0});
        }

        void Scene3d::drawCube(std::vector<std::vector<Point>> _faces){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix
            
        }


        void Scene3d::drawAxis(const Eigen::Matrix4f _pose){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix

            // X
            glEnable(GL_COLOR_MATERIAL);
            glColor4f(1.0f, 0.0f, 0.0f, 1.0f);

            glLineWidth((GLfloat)10.0);
            glBegin(GL_LINES);
                glVertex3f(_pose(0,3), _pose(1,3), _pose(2,3));
                glVertex3f( _pose(0,3) + _pose(0,0)*0.2, 
                            _pose(1,3) + _pose(1,0)*0.2,
                            _pose(2,3) + _pose(2,0)*0.2);
            glEnd();
            // Y
            glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
                glVertex3f(_pose(0,3), _pose(1,3), _pose(2,3));
                glVertex3f( _pose(0,3) + _pose(0,1)*0.2, 
                            _pose(1,3) + _pose(1,1)*0.2, 
                            _pose(2,3) + _pose(2,1)*0.2);
            glEnd();
            // Z
            glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
            glBegin(GL_LINES);
                glVertex3f(_pose(0,3), _pose(1,3), _pose(2,3));
                glVertex3f( _pose(0,3) + _pose(0,2)*0.2, 
                            _pose(1,3) + _pose(1,2)*0.2, 
                            _pose(2,3) + _pose(2,2)*0.2);
            glEnd();
            glLineWidth((GLfloat)1.0);
        }

        void Scene3d::drawOccupancyMap(){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix
            std::vector<Point> centers = centers_;
            float s2 = side_/2;

            for(const auto &c:centers_){
                drawCube({
                    {c.x-s2, c.y-s2, c.z+s2},
                    {c.x-s2, c.y+s2, c.z+s2},
                    {c.x+s2, c.y+s2, c.z+s2},
                    {c.x+s2, c.y-s2, c.z+s2},
                    {c.x-s2, c.y-s2, c.z-s2},
                    {c.x-s2, c.y+s2, c.z-s2},
                    {c.x+s2, c.y+s2, c.z-s2},
                    {c.x+s2, c.y-s2, c.z-s2}
                });
            }
        }

        bool Scene3d::getHeatMapColor(float value, float *red, float *green, float *blue) {
            const int NUM_COLORS = 4;
            static float color[NUM_COLORS][3] = { {0,0,1}, {0,1,0}, {1,1,0}, {1,0,0} };
                // A static array of 4 colors:  (blue,   green,  yellow,  red) using {r,g,b} for each.
            
            int idx1;        // |-- Our desired color will be between these two indexes in "color".
            int idx2;        // |
            float fractBetween = 0;  // Fraction between "idx1" and "idx2" where our value is.
            
            if(value <= 0)      {  idx1 = idx2 = 0;            }    // accounts for an input <=0
            else if(value >= 1)  {  idx1 = idx2 = NUM_COLORS-1; }    // accounts for an input >=0
            else
            {
                value = value * (NUM_COLORS-1);        // Will multiply value by 3.
                idx1  = floor(value);                  // Our desired color will be after this index.
                idx2  = idx1+1;                        // ... and before this index (inclusive).
                fractBetween = value - float(idx1);    // Distance between the two indexes (0-1).
            }
                
            *red   = (color[idx2][0] - color[idx1][0])*fractBetween + color[idx1][0];
            *green = (color[idx2][1] - color[idx1][1])*fractBetween + color[idx1][1];
            *blue  = (color[idx2][2] - color[idx1][2])*fractBetween + color[idx1][2];
            
            return true;
        }
    }
}