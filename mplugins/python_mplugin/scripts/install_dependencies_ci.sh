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

install_git_repo "pybind11"     "https://github.com/pybind/pybind11.git"
