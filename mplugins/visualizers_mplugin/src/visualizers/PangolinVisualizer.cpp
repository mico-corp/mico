//---------------------------------------------------------------------------------------------------------------------
//  Visualizers MICO plugin
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

#ifdef MICO_HAS_PANGOLIN

#if defined(_WIN32)
#   undef PANGOLIN_UNUSED
#   define PANGOLIN_UNUSED(x) (void)(x)
#endif

#include <mico/visualizers/PangolinVisualizer.h>

#include <pangolin/pangolin.h>
#include <pangolin/scene/axis.h>
#include <pangolin/scene/scenehandler.h>
#include <pangolin/gl/gltext.h>
#include <pangolin/gl/glfont.h>
#include <thread>
//#include <unistd.h>


namespace mico{
        int PangolinVisualizer::sWinId = 0;

        PangolinVisualizer::PangolinVisualizer(){
            windowName_ = "pangolin_"+std::to_string(sWinId);
            sWinId++;
            
            voxelFilter_.setLeafSize (0.01f, 0.01f, 0.01f);
        
            renderThread_ = std::thread(&PangolinVisualizer::renderCallback, this);    
        }

        PangolinVisualizer::~PangolinVisualizer(){
            running_ = false;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if(renderThread_.joinable())
                renderThread_.join();
        }

        void PangolinVisualizer::currentPose(const Eigen::Matrix4f &_pose){
            renderGuard_.lock();
            currentPose_ = _pose;
            renderGuard_.unlock();
        }

        void PangolinVisualizer::addLine(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector4f &_color){
            addLines({_p0, _p1}, _color);
        }

        void PangolinVisualizer::addLines(const std::vector<Eigen::Vector3f> &_pts, const Eigen::Vector4f &_color){
            renderGuard_.lock();
            linesToDraw_.push_back(_pts);
            colorLines_.push_back(_color);
            renderGuard_.unlock();
        }

        void PangolinVisualizer::addPointCloud(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud){
            renderGuard_.lock();
            absoluteCloud_ += *_cloud;
            voxelFilter_.setInputCloud (absoluteCloud_.makeShared());
            voxelFilter_.filter(absoluteCloud_);
            cloudsToDraw_.push_back(_cloud);
            renderGuard_.unlock();
        }

        void PangolinVisualizer::addText(const std::string &_text, const Eigen::Vector3f &_position){
            renderGuard_.lock();
            textToDraw_.push_back(_text);
            textPosition_.push_back(_position);
            renderGuard_.unlock();
        }
        
        void PangolinVisualizer::clearPointClouds(){
            renderGuard_.lock();
            cloudsToDraw_.clear();
            renderGuard_.unlock();
        }
        
        void PangolinVisualizer::clearLines(){
            renderGuard_.lock();
            linesToDraw_.clear();
            colorLines_.clear();
            renderGuard_.unlock();
        }
        


        void PangolinVisualizer::renderCallback(){
            pangolin::CreateWindowAndBind(windowName_,640,480);
            glEnable(GL_DEPTH_TEST);
        
            pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
                pangolin::ModelViewLookAt(-2,0,-2, 0,0,0, pangolin::AxisY)
            );

            pangolin::Renderable tree;
            tree.Add( std::make_shared<pangolin::Axis>() );

            // Create Interactive View in window
            pangolin::SceneHandler handler(tree, s_cam);
            pangolin::View& d_cam = pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -640.0f/480.0f)
                    .SetHandler(&handler);

            d_cam.SetDrawFunction([&](pangolin::View& view){
                view.Activate(s_cam);
                tree.Render();
            });

            // Ensure that blending is enabled for rendering text.
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

            while(running_ && !pangolin::ShouldQuit() ) {

                // Clear screen and activate view to render into
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

                drawCurrentPose();
                drawLines();
                drawPointClouds();
                drawText();

                // Swap frames and Process Events
                pangolin::FinishFrame();
                std::this_thread::sleep_for(std::chrono::microseconds(1000));
            }
            
            pangolin::DestroyWindow(windowName_);
        }

        void PangolinVisualizer::drawCurrentPose(){
            renderGuard_.lock();
            auto pose = currentPose_;
            renderGuard_.unlock();

            glLineWidth(2);
            // X
            glColor4f(1,0,0,1);
            glBegin(GL_LINES);
            glVertex3f(pose(0,3), pose(1,3), pose(2,3));
            glVertex3f( pose(0,3) + pose(0,0)*0.2, 
                        pose(1,3) + pose(1,0)*0.2, 
                        pose(2,3) + pose(2,0)*0.2);
            glEnd();
            // X

            glColor4f(0,1,0,1);
            glBegin(GL_LINES);
            glVertex3f(pose(0,3), pose(1,3), pose(2,3));
            glVertex3f( pose(0,3) + pose(0,1)*0.2, 
                        pose(1,3) + pose(1,1)*0.2, 
                        pose(2,3) + pose(2,1)*0.2);
            glEnd();
            
            // Z
            glColor4f(0,0,1,1);
            glBegin(GL_LINES);
            glVertex3f(pose(0,3), pose(1,3), pose(2,3));
            glVertex3f( pose(0,3) + pose(0,2)*0.2, 
                        pose(1,3) + pose(1,2)*0.2, 
                        pose(2,3) + pose(2,2)*0.2);
            glEnd();


        }

        void PangolinVisualizer::drawLines(){
            renderGuard_.lock();
            auto linesToDraw = linesToDraw_;
            auto colorLines = colorLines_;
            renderGuard_.unlock();
            for(unsigned j = 0; j < linesToDraw.size(); j++){
                auto &line = linesToDraw[j];
                auto &color = colorLines[j];
                glLineWidth(2);
                glColor4f(color[0], color[1], color[2], color[3]);
                glBegin(GL_LINES);
                for(unsigned i = 1; i < line.size(); i++){
                    glVertex3f(line[i-1][0], line[i-1][1], line[i-1][2]);
                    glVertex3f(line[i][0], line[i][1], line[i][2]);
                }
                glEnd();
            }
        }

        void PangolinVisualizer::drawPointClouds(){
            renderGuard_.lock();
            auto clouds = cloudsToDraw_;
            renderGuard_.unlock();

            // for(auto &cloud: clouds){
                glPointSize(1);
                glBegin(GL_POINTS);
                for(auto &p: absoluteCloud_.points){
                    glColor3f(p.r/255.0f,p.g/255.0f,p.b/255.0f);
                    glVertex3f(p.x,p.y,p.z);
                }
                glEnd();
            // }
        }

        void PangolinVisualizer::drawText(){
            renderGuard_.lock();
            auto text = textToDraw_;
            auto textPosition = textPosition_;
            renderGuard_.unlock();
            for(unsigned j = 0; j < text.size(); j++){
                pangolin::GlFont::I().Text(text[j].c_str()).Draw(textPosition[j](0),textPosition[j](1),textPosition[j](2));
            }

        }

}

#endif