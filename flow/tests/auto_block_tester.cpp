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
#include <flow/Outpipe.h>

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
std::uniform_int_distribution<int> distr(0, 5); // define the range

void randomizeParameter(flow::ConfigParameterDef &_param, bool _verbose = false) {
	int rndNumber = distr(gen);
	switch (rndNumber) {
	case 0: // set as bool
		_param.type_ = flow::ConfigParameterDef::eParameterType::BOOLEAN;
		_param.value_ = bool(rand() & 2);
		if (_verbose) std::cout << "boolean\n";
		break;
	case 1: // set as decimal
		_param.type_ = flow::ConfigParameterDef::eParameterType::DECIMAL;
		_param.value_ = float(rand())/rand();
		if (_verbose) std::cout << "integer\n";
		break;
	case 2: // set as integer
		_param.type_ = flow::ConfigParameterDef::eParameterType::INTEGER;
		_param.value_ = int(rand());
		if (_verbose) std::cout << "decimal\n";
		break;
	case 3: // set as options
		_param.type_ = flow::ConfigParameterDef::eParameterType::OPTIONS;
		_param.selectedOption_ = std::string("A");
		_param.value_ = std::vector{ std::string("A") , std::string("B") , std::string("C") };
		if (_verbose) std::cout << "options\n";
		break;
	case 4: // set as path
	{
		_param.type_ = flow::ConfigParameterDef::eParameterType::PATH;
		_param.value_ = fs::path("./no/I/am/not/our/path");
		if (_verbose) std::cout << "path\n";
		break;
	}
	case 5: // set as string
	{
		_param.type_ = flow::ConfigParameterDef::eParameterType::STRING;
		_param.value_ = std::string("HEY I AM NOT WHAT U WANTED");
		if (_verbose) std::cout << "string\n";
		break;
	}
	}

	if(_verbose)
		std::cout.flush();

}


TEST(create_blocks, randomization_parameters) {
	for (unsigned i = 0; i < 100; i++) {
		flow::ConfigParameterDef param;
		randomizeParameter(param, false);
		EXPECT_NO_THROW(param.serialize());
	}

}


// Try to create all the blocks. None of the constructors should crash, neither their configuration with empty values.
TEST(create_blocks, create_blocks_destroy) {
	for (auto& [tag, creator] : creatorsHolder.creators_) {
		std::cout << "---------------------------------------------------------------------" << std::endl;

		// Create block
		auto block = creator();
		std::cout << "Testing construction and destruction of block: " << block->name() << std::endl;
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
				std::string serializedParam = "";
				EXPECT_NO_THROW(serializedParam = param.serialize());
				std::cout << "\t" + serializedParam << std::endl;
			}
			std::cout << "Test " << i;
			std::cout.flush();
			EXPECT_NO_THROW(block->configure(defaultParameters));
			std::cout << "---->  OK " << std::endl;
		}
	}
}


std::pair<flow::Outpipe*, std::function<void()>> randomPipe() {
	int rndNumber = distr(gen);

	Outpipe* ptr;
	std::function<void()> fn;
	switch (rndNumber)
	{
	case 0:
		ptr = flow::makeOutput<bool>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(true); }, ptr);
	case 1:
		ptr = flow::makeOutput<int>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(123); }, ptr);
	case 2:
		ptr = flow::makeOutput<float>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(1.023f); }, ptr);
	case 3:
		ptr = flow::makeOutput<std::string>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(std::string("hola")); }, ptr);
	case 4:
		ptr = flow::makeOutput<std::vector<float>>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(std::vector<float>{1.30f, 1.30f, 1.30f}); }, ptr);
	case 5:
		ptr = flow::makeOutput<std::vector<int>>("output");
		fn = std::bind([](Outpipe* _ptr) { _ptr->flush(std::vector<int>{2, 3, 1, 2}); }, ptr);
	}
	
	return { ptr, fn };
}

TEST(input_tests, random_inputs) {

	for (auto& [tag, creator] : creatorsHolder.creators_) {
		std::cout << "---------------------------------------------------------------------" << std::endl;
		auto block = creator();
		std::cout << "Testing random inputs in block: " << block->name() << std::endl;

		// Some blocks do no have any input policy. 
		if (block->getPolicy()) {
			auto inputs = block->getPolicy()->inputTags();
			std::vector<std::pair<flow::Outpipe*, std::function<void()>>> pipes;
			for (auto &input:inputs) {
				pipes.push_back(randomPipe());
				pipes.back().first->registerPolicy(block->getPolicy(), input);
			}

			for (unsigned i = 0; i < 100; i++) {
				for (auto& [pipe, cb] : pipes) {
					cb();
				}
			}
		}
		// Wait until all threads are idle....
		while (flow::ThreadPool::get()->loadRatio() != 0.0f) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 666 TODO extra time in case a thread is finishing the task
	}
}

#include "InputGenerators.h"

TEST(input_tests, adequate_inputs) {

	for (auto& [tag, creator] : creatorsHolder.creators_) {
		std::cout << "---------------------------------------------------------------------" << std::endl;
		auto block = creator();
		std::cout << "Testing adequate inputs in block: " << block->name() << std::endl;

		// Some blocks do no have any input policy. 
		if (block->getPolicy()) {
			auto inputs = block->getPolicy()->inputs();
			
			std::vector<std::pair<flow::Outpipe*, InputGenerator*>> pipes;
			bool hasError = false;
			for (auto& input : inputs) {
				auto *pipe = new Outpipe("out", input.typeName());
				auto *iGen = InputGenerator::create(input.typeName());
				if (!iGen) { // No generator available, can't test it
					hasError = true;
					break;
				}
				pipes.push_back({pipe, iGen});
				pipes.back().first->registerPolicy(block->getPolicy(), input.tag());
			}

			if (hasError) continue;	// Error configuring the test, skip this block


			std::cout << "\t Default constructible input" << std::endl;
			// Test default constructible
			for (unsigned i = 0; i < 100; i++) {
				for (auto& [pipe, gen] : pipes) {
					pipe->flush(gen->defaultContructible());
				}
			}

			// Test random inputs
			std::cout << "\t Random input generation" << std::endl;
			for (unsigned i = 0; i < 100; i++) {
				for (auto& [pipe, gen] : pipes) {
					pipe->flush(gen->randomInput());
				}
			}
		}
		// Wait until all threads are idle....
		while (flow::ThreadPool::get()->loadRatio() != 0.0f) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 666 TODO extra time in case a thread is finishing the task
	}
}


int main(int _argc, char** _argv) {
	QApplication app(_argc, _argv);

	testing::InitGoogleTest(&_argc, _argv);
	return RUN_ALL_TESTS();

}