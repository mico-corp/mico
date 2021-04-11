##---------------------------------------------------------------------------------------------------------------------
##  MICO TEMPLATE plugin
##---------------------------------------------------------------------------------------------------------------------
##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92) & Marco Montes Grova (a.k.a mgrova)
##---------------------------------------------------------------------------------------------------------------------
##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
##  and associated documentation files (the "Software"), to deal in the Software without restriction,
##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
##  furnished to do so, subject to the following conditions:
##
##  The above copyright notice and this permission notice shall be included in all copies or substantial
##  portions of the Software.
##
##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
##---------------------------------------------------------------------------------------------------------------------

build_directory=%1
install_directory=%2

git clone https://github.com/opencv/opencv $build_directory/opencv
git clone https://github.com/opencv/opencv_contrib $build_directory/opencv_contrib

cd $build_directory/opencv_contrib
git checkout 4.2.0

cd $build_directory/opencv
git checkout 4.2.0

mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX=$install_directory -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF  -DOPENCV_EXTRA_MODULES_PATH=$build_directory/opencv_contrib/modules
cmake --build . --config Release -j
cmake --build . --config Release -j --target INSTALL
cmake --build . --config debug -j
cmake --build . --config debug -j --target INSTALL