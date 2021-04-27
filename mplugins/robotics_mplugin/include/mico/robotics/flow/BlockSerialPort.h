//---------------------------------------------------------------------------------------------------------------------
//  robotics MICO plugin
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



#ifndef MICO_ROBOTICS_FLOW_BLOCKS_BLOCKSERIALPORT_H_
#define MICO_ROBOTICS_FLOW_BLOCKS_BLOCKSERIALPORT_H_

#include <flow/Block.h>
#include <flow/flow.h>
#include <mico/robotics/SerialPort.h>


/*
namespace mico{
    class BlockSerialPort:public flow::Block{
    public:
        /// Get name of block
        virtual std::string name() const override {return "SerialPort";}        
        /// Retreive icon of block    
            virtual QIcon icon() const override { 
            std::string userDir(getenv("USER"));
            std::string resourcesDir = "/home/"+userDir+"/.flow/plugins/resources/robotics/";
            return QIcon((resourcesDir+"serial_port_icon.png").c_str());
        }
        BlockSerialPort();
        ~BlockSerialPort();

        /// Configure block with given parameters.
        virtual bool configure(std::vector<flow::ConfigParameterDef> _params) override;
        /// Get list of parameters of the block
        std::vector<flow::ConfigParameterDef> parameters() override;

        /// Returns a brief description of the block
        std::string description() const override {return    "Creates a serial port"
                                                            "   - Inputs: \n"
                                                            "   - Outputs: \n";};

    private:
        std::string device_;
        SerialPort *sp_;
        int baudrate_;
        bool run_ = true;
        std::thread readThread_;
    };

    //--------------------------------------------------------------------------------------------------------
    // Data cast blocks
    template<typename T_, char const* BlockName_, char const* InputName_, char const* InputType_>
    class BlockCastToBytes: public flow::Block{
    public:
        BlockCastToBytes(){
            createPipe("bytes","byte");

            createPolicy({{{InputName_, InputType_}}});

            registerCallback({InputName_}, 
                                [&](flow::DataFlow _data){
                                        if(idle_){
                                            idle_ = false;
                                                auto data = _data.get<T_>(InputName_);  
                                                uint8_t * dataPtr = (uint8_t*)&data;
                                                for(unsigned i = 0; i < sizeof(T_); i++){
                                                    getPipe("bytes")->flush(*dataPtr);
                                                    dataPtr++;

                                                }
                                            idle_ = true;
                                        }
                                    }
                                );
        }

        /// Get name of block
        virtual std::string name() const override {return BlockName_;}        
    
    protected:
        bool idle_ = true;
    };

    char const BlockNameFloatToBytes[] = "Float to Bytes";
    char const nameFloatToBytes[] = "float";
    char const typeFloatToBytes[] = "float";
    typedef BlockCastToBytes<float, BlockNameFloatToBytes, nameFloatToBytes, typeFloatToBytes>  BlockFloatToBytes;


}
*/


#endif