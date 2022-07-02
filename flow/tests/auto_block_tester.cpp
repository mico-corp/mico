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

#include <random>

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

std::random_device rd; // obtain a random number from hardware
std::mt19937 gen(rd()); // seed the generator
std::uniform_int_distribution<int> distr(0, 6); // define the range

void randomizeParameter(flow::ConfigParameterDef &_param) {
	switch (distr(gen)) {
	case 0: // set as bool
		_param.type_ = flow::ConfigParameterDef::eParameterType::BOOLEAN;
		_param.value_ = rand() & 2;
		break;
	case 1: // set as decimal
		_param.type_ = flow::ConfigParameterDef::eParameterType::DECIMAL;
		_param.value_ = float(rand())/rand();
		break;
	case 2: // set as integer
		_param.type_ = flow::ConfigParameterDef::eParameterType::INTEGER;
		_param.value_ = rand();
		break;
	case 3: // set as options
		_param.type_ = flow::ConfigParameterDef::eParameterType::OPTIONS;
		_param.selectedOption_ = "";
		for (int i = 0; i < rand() % 16; i++) {
			_param.selectedOption_ += char(rand() % 60 + 50);
		}
		_param.value_ = _param.selectedOption_;
		break;
	case 4: // set as path
	{
		_param.type_ = flow::ConfigParameterDef::eParameterType::PATH;
		std::string randomPath = "";
		for (int i = 0; i < rand() % 50; i++) {
			randomPath += char(rand() % 60 + 50);
		}
		_param.value_ = fs::path(randomPath);
		break;
	}
	case 5: // set as string
	{
		_param.type_ = flow::ConfigParameterDef::eParameterType::STRING;
		std::string randomStr = "";
		for (int i = 0; i < rand() % 50; i++) {
			randomStr += char(rand() % 60 + 50);
		}
		_param.value_ = randomStr;
	}
		break;
	}
}

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
		auto defaultParameters = block->parameters();
		for (auto& param : defaultParameters) {
			std::cout << "\t" + param.serialize() << std::endl;
		}
		std::cout << "Configuration with default list of parameters ";
		std::cout.flush();
		EXPECT_NO_THROW(block->configure(defaultParameters));
		std::cout << "---->  OK " << std::endl;

		// Initialize with random parameters. 10 times
		for (unsigned i = 0; i < 5;i++) {
			for (auto& param : defaultParameters) {
				randomizeParameter(param);
			}

			for (auto& param : defaultParameters) {
				std::cout << "\t" + param.serialize() << std::endl;
			}
			std::cout.flush();
			EXPECT_NO_THROW(block->configure(defaultParameters));
		}
	}
}


int main(int _argc, char** _argv) {
	QApplication app(_argc, _argv);

	testing::InitGoogleTest(&_argc, _argv);
	return RUN_ALL_TESTS();
}