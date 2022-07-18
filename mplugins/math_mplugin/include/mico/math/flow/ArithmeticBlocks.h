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



#ifndef MICO_FLOW_BLOCKS_ARITHMETICBLOCKS_H_
#define MICO_FLOW_BLOCKS_ARITHMETICBLOCKS_H_

#include <flow/Block.h>
#include <QPushButton>

namespace mico{
    namespace math{

        /// Mico block that performs a mathematical addition on the two given inputs
        /// @ingroup  mico_math
        class BlockSum:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Sum";}        
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"plus-positive-add-mathematical-symbol.svg").string();
            }

            /// Base constructor
            BlockSum();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Sum two float numbers"
                                                                "   - Outputs: addition of both inputs\n";};

        };

        /// Mico block that performs a mathematical substraction on the two given inputs
        /// @ingroup  mico_math
        class BlockSubstract:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Substract";}  
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"minus-sign.svg").string();
            }      

            // Base constructor
            BlockSubstract();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Subsctract two float numbers"
                                                                "   - Outputs: Substraction of both inputs\n";};

        };


        /// Mico block that performs a mathematical scalar multiplication on the two given inputs
        /// @ingroup  mico_math 
        class BlockMultiply:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Multiply";}  
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"multiply-mathematical-sign.svg").string();
            }     

            /// Base constructor 
            BlockMultiply();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Multiply two float numbers"
                                                                "   - Outputs: Multiplication of both inputs\n";};

        };


        /// Mico block that performs a mathematical scalar division on the two given inputs
        /// @ingroup  mico_math 
        class BlockDivide:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Divide";}      
            
            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"divide-mathematical-sign.svg").string();
            }         

            /// Base constructor
            BlockDivide();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Divide two float numbers"
                                                                "   - Outputs: division of both inputs\n";};

        };


        /// Mico block that performs a mathematical square root on the given input
        /// @ingroup  mico_math 
        class BlockSquareRoot:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "sqrt";}    

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"square-root-mathematical-symbol.svg").string();
            }

            /// Base constructor
            BlockSquareRoot();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Square root of given input"
                                                                "   - Outputs: Square root of given input\n";};

        };


        /// Mico block that performs a mathematical pow with the given inputs
        /// @ingroup  mico_math 
        class BlockPow:public flow::Block{
        public:
            /// Get name of block
            std::string name() const override {return "Pow";}    

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir()/"math"/"pow-symbol.svg").string();
            }
            /// Base constructor
            BlockPow();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "Pow of first number by second number"
                                                                "   - Outputs: pow\n";};

        };

        /// Mico block that performs an integration (accumulation) of the input signal
        /// @ingroup  mico_math 
        class BlockIntegrator :public flow::Block {
        public:
            /// Get name of block
            std::string name() const override { return "Integrator"; }

            /// Base constructor
            BlockIntegrator();

            /// Retreive icon of block    
            std::string icon() const override {
                return (flow::Persistency::resourceDir() / "math" / "integral_block_icon.svg").string();
            }

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Custom widget
            QWidget* customWidget() { return resetBt_; };

            /// Returns a brief description of the block
            std::string description() const override {
                return      "Accumulate the input signal"
                    "   - Input: signal\n"
                "   - Outputs: accumulation\n";
            };

        private:
            float acc_ = 0.0f;
            QPushButton* resetBt_;
        };

    /// Mico block that performs an integration (accumulation) of the input signal
        /// @ingroup  mico_math 
    class BlockDerivative :public flow::Block {
    public:
        /// Get name of block
        std::string name() const override { return "Derivative"; }

        /// Base constructor
        BlockDerivative();

        /// Retreive icon of block    
        std::string icon() const override {
            return (flow::Persistency::resourceDir() / "math" / "derivative_block_icon.svg").string();
        }

        /// Return if the block is configurable.
        bool isConfigurable() override { return false; };

        /// Custom widget
        QWidget* customWidget() { return resetBt_; };

        /// Returns a brief description of the block
        std::string description() const override {
            return      "Computes derivative of signal"
                "   - Input: signal\n"
            "   - Outputs: derivative\n";
        };

    private:
        bool isFirst_ = true;
        float lastVal_;
        std::chrono::time_point<std::chrono::high_resolution_clock> t0_;
        QPushButton* resetBt_;
    };
    }
}



#endif
