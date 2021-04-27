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



#include <mico/ar/gl_helpers/Mesh.h>
#include <mico/ar/gl_helpers/stl_reader.h>

#if defined(WIN32)
#   include <GL/freeglut.h> // GLUT, include glu.h and gl.h
#elif defined(linux)
#   include <GL/glut.h> // GLUT, include glu.h and gl.h
#endif

#include <iostream>
#ifndef M_PI
# define M_PI 3.14159265359 
#endif

namespace mico {
    namespace ar {
        bool Mesh::loadMesh(std::string _path){
            meshReader_ = new stl_reader::StlMesh<float, unsigned int>(_path);
            return meshReader_->num_vrts()>0;
        }

        void Mesh::transform(Eigen::Matrix4f _t){
            translation_ = _t.block<3,1>(0,3);
            rpy_ = _t.block<3,3>(0,0).eulerAngles(0, 1, 2); 
            pose_ = _t;
        }

        void Mesh::displayMesh(){
            glMatrixMode(GL_MODELVIEW);                         // To operate on model-view matrix
            glLoadIdentity();                       // Reset the model-view matrix
            
            glTranslatef(translation_[0], translation_[1], translation_[2]);
            glRotatef(rpy_[0]/M_PI*180.0f, 1,0,0);
            glRotatef(rpy_[1]/M_PI*180.0f, 0,1,0);
            glRotatef(rpy_[2]/M_PI*180.0f, 0,0,1);
            
            setMaterial({r_, g_, b_, a_}, {r_, g_, b_, a_}, {0,0,0,0}, 1.0f);
            
            glBegin(GL_TRIANGLES);
            for(size_t itri = 0; itri < meshReader_->num_tris(); ++itri) {
                const float* n = meshReader_->tri_normal (itri);
                for(size_t icorner = 0; icorner < 3; ++icorner) {
                    const float* c = meshReader_->vrt_coords (meshReader_->tri_corner_ind (itri, icorner));
                    glNormal3f(n[0], n[1], n[2]);
                    glVertex3f(c[0], c[1], c[2]);
            }
            }
            glEnd();
        }

        void Mesh::setColor(float _r, float _g, float _b, float _a){
            r_ = _r;
            g_ = _g;
            b_ = _b;
            a_ = _a;
        }

        void Mesh::setMaterial(std::vector<float> _ambient, std::vector<float> _diffuse, std::vector<float> _specular, float _shininess){
            glColor4f(_ambient[0], _ambient[1], _ambient[2], _ambient[3]);
            glMaterialfv(GL_FRONT, GL_AMBIENT, _ambient.data());
            glMaterialfv(GL_FRONT, GL_DIFFUSE, _diffuse.data());
            glMaterialfv(GL_FRONT, GL_SPECULAR, _specular.data());
            glMaterialf(GL_FRONT, GL_SHININESS, _shininess);
        }
    }
}