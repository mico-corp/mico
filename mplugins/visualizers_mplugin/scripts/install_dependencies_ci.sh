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

install_git_repo "nodeeditor"   "https://github.com/mico-corp/nodeeditor.git"
install_git_repo "flow"         "https://github.com/mico-corp/flow.git"
install_git_repo "core_mplugin" "https://github.com/mico-corp/core_mplugin.git"

sudo apt-get install -y liblapack-dev libblas-dev libopenblas-dev libeigen3-dev
sudo apt-get install -y libomp-dev libsuitesparse-dev libcholmod3

install_git_repo "DBoW2"         "https://github.com/dorian3d/DBoW2.git"
install_git_repo "DLoopDetector" "https://github.com/dorian3d/DLoopDetector.git"
install_git_repo "g2o"           "https://github.com/RainerKuemmerle/g2o.git"
install_git_repo "slam_mplugin"  "https://github.com/mico-corp/slam_mplugin.git"

install_git_repo "Pangolin"     "https://github.com/stevenlovegrove/Pangolin.git"