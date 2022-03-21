//---------------------------------------------------------------------------------------------------------------------
//  Cameras wrapper MICO plugin
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



#ifndef MICO_FLOW_BLOCKS_FUNCTIONBLOCKS_H_
#define MICO_FLOW_BLOCKS_FUNCTIONBLOCKS_H_

#include <flow/Block.h>
#include <flow/Persistency.h>
class QLineEdit;

namespace mico{

    namespace math{

        
        /// Mico block that generates a stream of a timer
        /// @ingroup  mico_math 
        class BlockTimer:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Timer";}     
            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"math"/"timer-symbol.svg").string().c_str());
            }

            
            BlockTimer();
            
            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Stream a timer. Resolution is in seconds"
                                                                "   - Outputs: Time counter\n";};

        protected:
            void loopCallback() override;

        private:
            float resolution_ = 0.001; // FPS

            QLineEdit *valueView_;
        };


        /// Mico block that generates a sinusoidal function given a time entry function.
        /// @ingroup  mico_math 
        class BlockSine:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Sine";}   

            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"math"/"sine-symbol.svg").string().c_str());
            }
            
            /// Base constructor
            BlockSine();
            
            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;
            
            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Stream a constant by a given rate."
                                                                "   - Outputs: Constant value\n";};
            
        private:
            float freq_ = 0;
            float amplitude_ = 0;
            float phase_ = 0;
        };


    
        /// Mico block that generates a sinusoidal (cosine) function given a time entry function.
        /// @ingroup  mico_math 
        class BlockCosine:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Cosine";}   
            /// Retreive icon of block    
            QIcon icon() const override { 
                return QIcon((flow::Persistency::resourceDir()/"math"/"cosine-symbol.svg").string().c_str());
            }
            
            BlockCosine();
            
            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;
            
            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Stream a constant by a given rate."
                                                                "   - Outputs: Constant value\n";};

        private:
            float freq_ = 0;
            float amplitude_ = 0;
            float phase_ = 0;
        };

        /// Mico block that linearly maps a signal.
        /// @ingroup  mico_math 
        class BlockLinearMap :public flow::Block {
        public:
            /// Get name of block
            std::string name() const override { return "Linear Map"; }
            
            /// Retreive icon of block    
            /*QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "math" / "cosine-symbol.svg").string().c_str());
            }*/

            BlockLinearMap();

            /// Configure block with given parameters.
            bool configure(std::vector<flow::ConfigParameterDef> _params) override;

            /// Get list of parameters of the block
            std::vector<flow::ConfigParameterDef> parameters() override;

            /// Return if the block is configurable.
            bool isConfigurable() override { return true; };

            /// Returns a brief description of the block
            std::string description() const override {
                return    "Function block."
                    "   - Input: float number\n";
                    "   - Outputs: float number\n";
            };

        private:
            float minInput_ = 0;
            float maxInput_ = 1024;
            float minOutput_ = 0;
            float maxOutput_ = 180;
        };
    }
}



#endif