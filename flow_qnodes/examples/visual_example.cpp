//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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


#include <flow/visual/FlowVisualInterface.h>
#include <flow/visual/blocks/FlowVisualBlock.h>
#include <flow/DataFlow.h>

#include <flow/flow.h>
#include <string>

#include <sstream>

using namespace flow;

class FloatStreamerBlock: public Block{
public:
    std::string name() {return "Float streamer";}
        FloatStreamerBlock(){
            createPipe("time", "float");
        }

        virtual void loopCallback() override{
            auto t0 = std::chrono::high_resolution_clock::now();
            while(isRunningLoop()){
                auto t1 = std::chrono::high_resolution_clock::now();
                float diff = counter_ + std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0f;
                getPipe("time")->flush(diff);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }      
        }

        /// Configure block with given parameters.
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override { 
            if(_params["init_value"] != ""){
                std::istringstream istr(_params["init_value"]);
                istr >> counter_;
                return true;
            }else{
                return false;
            } 
        };
        virtual /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override{ return {"init_value"}; };
private:
    float counter_  = 1.0;
};

class FloatCouterBlock: public Block{
public:
    std::string name() {return "Float Couter";}
    FloatCouterBlock(){
        createPolicy({{{"clock", "float"}}});
        registerCallback({"clock"}, 
                            [&](DataFlow _data){
                                float data = _data.get<float>("clock");
                                std::cout << data << std::endl;
                            }
                            );
    }
    ~FloatCouterBlock(){ /*Nothing spetial*/}
};


class IntStreamerBlock: public Block{
public:
    std::string name() {return "Int streamer";}
        IntStreamerBlock(){
            createPipe("time", "int");
        }

        virtual void loopCallback() override{
            auto t0 = std::chrono::high_resolution_clock::now();
            while(isRunningLoop()){
                auto t1 = std::chrono::high_resolution_clock::now();
                float diff = counter_ + std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0f;
                getPipe("time")->flush(diff);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }      
        }

        /// Configure block with given parameters.
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override { 
            if(_params["init_value"] != ""){
                std::istringstream istr(_params["init_value"]);
                istr >> counter_;
                return true;
            }else{
                return false;
            } 
        };
        virtual /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override{ return {"init_value"}; };
private:
    int counter_  = 1;
};

class IntCouterBlock: public Block{
public:
    std::string name() {return "Int Couter";}
    IntCouterBlock(){
        createPolicy({{{"clock", "int"}}});
        registerCallback({"clock"}, 
                            [&](DataFlow _data){
                                int data = _data.get<int>("clock");
                                std::cout << data << std::endl;
                            }
                            );
    }
    ~IntCouterBlock(){ /*Nothing spetial*/}
};




int main(int _argc, char **_argv){

    FlowVisualInterface::RegistryFnType_ fn = [](FlowVisualInterface::RegistryType_ &_ret){
        _ret->registerModel<FlowVisualBlock<FloatStreamerBlock, true>>("streamer");
        _ret->registerModel<FlowVisualBlock<FloatCouterBlock>>("visualizer");
        _ret->registerModel<FlowVisualBlock<IntStreamerBlock, true>>("streamer");
        _ret->registerModel<FlowVisualBlock<IntCouterBlock>>("visualizer");
    };


    FlowVisualInterface interface;
    interface.setNodeRegisterFn(fn);
    interface.init(_argc, _argv);

}