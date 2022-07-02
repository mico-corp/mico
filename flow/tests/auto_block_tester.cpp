//---------------------------------------------------------------------------------------------------------------------
//  FLOW
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91) 
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

#include <flow/plugins/PluginsLoader.h>

#include <QApplication> // 666 TODO Remove this in the future

#include <gtest/gtest.h>

#if defined(_MSC_VER)
	#include <Windows.h>
#endif

using namespace flow;

std::filesystem::path getExePath() {
	#if defined(_MSC_VER)
		wchar_t path[FILENAME_MAX] = { 0 };
		GetModuleFileNameW(nullptr, path, FILENAME_MAX);
		return std::filesystem::path(path);
	#else
		char path[FILENAME_MAX];
		ssize_t count = readlink("/proc/self/exe", path, FILENAME_MAX);
		return std::filesystem::path(std::string(path, (count > 0) ? count : 0));
	#endif
}

class CreatorsHolder {
public:
	CreatorsHolder() {
		PluginsLoader pl;

		fs::path exeDir = getExePath();
		fs::path binDir = exeDir.parent_path();

		// load all creators.
		creators_ = pl.parseFolder(binDir.string());
	}

	PluginNodeCreator::ListCreators creators_;
};

static CreatorsHolder creatorsHolder = CreatorsHolder();

// Try to create all the blocks. None of the constructors should crash, neither their configuration with empty values.
TEST(create_blocks, create_blocks) {
	
	for (auto& [tag, creator] : creatorsHolder.creators_) {
		std::cout << "---------------------------------------------------------------------" << std::endl;

		// Create block
		auto block = creator();
		std::cout << "Testing construction of block: " << block->name() << std::endl;

		// Configure with bad values
		std::cout << "Configuration with empty list of parameters ";
		EXPECT_NO_THROW(block->configure({}));
		std::cout << "---->  OK " << std::endl;

		// Get default parameters
		std::cout << "Configuration with default list of parameters ";
		auto defaultParameters = block->parameters();
		EXPECT_NO_THROW(block->configure(defaultParameters));
		std::cout << "---->  OK " << std::endl;
	}
}


int main(int _argc, char** _argv) {
	QApplication app(_argc, _argv);

	testing::InitGoogleTest(&_argc, _argv);
	return RUN_ALL_TESTS();
}