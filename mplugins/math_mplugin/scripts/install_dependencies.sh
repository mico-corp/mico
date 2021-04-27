#!/bin/sh

install_git_repo () {

	if [ -d "./$1" ] 
	then
		echo "Library $1 already installed" 
	else
		git clone $2
		cd $1
		if [ -z "$3" ]   # Is parameter #1 zero length?
		then
			git checkout "$3"
		fi
		
		mkdir build ; cd build
		cmake ..
		make -j$(nproc)
		sudo make install 
		cd ../..
	fi
}

bash install_librealsense2.sh

mkdir ~/.mico/ -p
cd ~/.mico/
mkdir thirdparty_plugins/cameras -p
cd thirdparty_plugins/cameras

install_git_repo "libfreenect"  "https://github.com/OpenKinect/libfreenect.git"
