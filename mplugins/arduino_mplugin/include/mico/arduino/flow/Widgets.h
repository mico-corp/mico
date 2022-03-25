//---------------------------------------------------------------------------------------------------------------------
//  Arduino MICO plugin
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



#ifndef MICO_ARDUINO_FLOW_WIDGETS_H_
#define MICO_ARDUINO_FLOW_WIDGETS_H_

#include <flow/Block.h>
#include <flow/Outpipe.h>

class QPushButton;
class QSlider;
class QLabel;
class QGroupBox;

namespace mico{

    namespace arduino {
        /// Mico interactive block that simulates a toggle button.
        /// @ingroup  mico_arduino
        class ToggleButtonBlock:public flow::Block{
        public:
            /// Retrieve name of block
            std::string name() const override {return "Toggle Button";}     
            
            /// Get custom view widget to be display in the graph
            QWidget * customWidget() override;

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "block_toggle.svg").string().c_str());
            }

            
            /// Base constructor
            ToggleButtonBlock();

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "ToggleButton\n";};
        private:
            QPushButton *button_;
        };


        /// Mico interactive block that simulates a slider bar to generate integer values.
        /// @ingroup  mico_arduino
        class SliderPwm :public flow::Block {
        public:
            /// Base constructor
            SliderPwm();

            /// Retrive name of block
            std::string name() const override { return "Slider Pwm"; }

            /// Get custom view widget to be display in the graph
            QWidget* customWidget() override;

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "block_slider.svg").string().c_str());
            }

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override { return    "Slider pwm\n"; };
        private:
            QSlider* slider_;
        };

        /// Mico block that switches between two different signals from a boolean input.
        /// @ingroup  mico_arduino
        class SignalSwitcher :public flow::Block {
        public:
            /// Base constructor
            SignalSwitcher();

            /// Retreive name of block
            std::string name() const override { return "Signal Switcher"; }

            /// Get custom view widget to be display in the graph
            QWidget* customWidget() override;

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "switch_A.svg").string().c_str());
            }

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override { return    "Signal Switcher\n"; };
        private:
            bool flowA_ = true;
            QLabel* img_;
            QGroupBox* customWidget_;
            std::string fileA = (flow::Persistency::resourceDir() / "arduino" / "switch_A.png").string();
            std::string fileB = (flow::Persistency::resourceDir() / "arduino" / "switch_B.png").string();
        };

        /// Mico block that performs a NOT operation on the given input.
        /// @ingroup  mico_arduino
        class NotOperator :public flow::Block{
        public:

            /// Base constructor
            NotOperator(){
                createPipe<bool>("NoA");
                
                createPolicy({  flow::makeInput<bool>("A")});

                registerCallback({"A"}, 
                    [&](flow::DataFlow _data){
                        bool res = !_data.get<bool>("A");
                        getPipe("NoA")->flush(res);
                    }
                );
            }


            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "block_not.svg").string().c_str());
            }

            /// Get name of block
            std::string name() const override {return "NOT";}     

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };
            
            /// Returns a brief description of the block
            std::string description() const override {return    "NOT\n";};
        };

        
        /// Mico block that performs a AND operation on the given input.
        /// @ingroup  mico_arduino
        class AndOperator :public flow::Block{
        public:
            /// Base constructor
            AndOperator(){
                createPipe<bool>("out");
                createPolicy({  flow::makeInput<bool>("A"),
                                flow::makeInput<bool>("B") });
                registerCallback({"A", "B"}, 
                    [&](flow::DataFlow _data){
                        bool res = _data.get<bool>("A") && _data.get<bool>("B");
                        getPipe("out")->flush(res);
                    }
                );
            }

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "block_and.svg").string().c_str());
            }

            /// Get name of block
            std::string name() const override {return "AND";}     
            
            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "AND\n";};
        };

        /// Mico block that performs a OR operation on the given input.
        /// @ingroup  mico_arduino
        class OrOperator :public flow::Block{
        public:
            /// Base constructor
            OrOperator(){
                createPipe<bool>("out");
                createPolicy({  flow::makeInput<bool>("A"),
                                flow::makeInput<bool>("B") });
                registerCallback({"in"}, 
                    [&](flow::DataFlow _data){
                        bool res = _data.get<bool>("A") || _data.get<bool>("B");
                        getPipe("out")->flush(res);
                    }
                );
            }

            /// Retreive icon of block    
            QIcon icon() const override {
                return QIcon((flow::Persistency::resourceDir() / "arduino" / "block_or.svg").string().c_str());
            }

            /// Get name of block
            std::string name() const override {return "OR";}     

            /// Return if the block is configurable.
            bool isConfigurable() override { return false; };

            /// Returns a brief description of the block
            std::string description() const override {return    "OR\n";};
        };

    }
}



#endif