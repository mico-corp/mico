@REM ##---------------------------------------------------------------------------------------------------------------------
@REM ##  MICO TEMPLATE plugin
@REM ##---------------------------------------------------------------------------------------------------------------------
@REM ##  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com & Ricardo Lopez Lopez (a.k.a Ric92) & Marco Montes Grova (a.k.a mgrova)
@REM ##---------------------------------------------------------------------------------------------------------------------
@REM ##  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
@REM ##  and associated documentation files (the "Software"), to deal in the Software without restriction,
@REM ##  including without limitation the rights to use, copy, modify, merge, publish, distribute,
@REM ##  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
@REM ##  furnished to do so, subject to the following conditions:
@REM ##
@REM ##  The above copyright notice and this permission notice shall be included in all copies or substantial
@REM ##  portions of the Software.
@REM ##
@REM ##  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
@REM ##  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
@REM ##  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
@REM ##  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
@REM ##  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
@REM ##---------------------------------------------------------------------------------------------------------------------

set build_directory=%1
set install_directory=%2

@REM ##---------------------------------------------------------------------------------------------------------------------
git clone https://github.com/zaphoyd/websocketpp %build_directory%/websocketpp

cd %build_directory%/websocketpp

mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX=%install_directory%
make -j4
sudo make install

@REM ##---------------------------------------------------------------------------------------------------------------------
cd %build_directory%
git clone https://github.com/Bardo91/fastcom %build_directory%/fastcom

cd %build_directory%/fastcom

mkdir build
cd build

cmake .. -DCMAKE_INSTALL_PREFIX=%install_directory% -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
make -j4
sudo make install