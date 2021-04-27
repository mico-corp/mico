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

#ifndef MICO_FLOW_ML_DETECTION_H_
#define MICO_FLOW_ML_DETECTION_H_

#include <opencv2/opencv.hpp>

#include <flow/Block.h>


class QComboBox;
class QSpinBox;

namespace mico{
    namespace ml{
        struct Detection{
            int label_;
            cv::Rect bb_;
        };

        /// Mico block that triggers a boolean signal if a detection of a given ID appears.
        /// @ingroup  mico_ml
        class BlockTriggerDetection:public flow::Block{
        public:
            /// Get name of block
            virtual std::string name() const override {return "Trigger on detection";}
            
            BlockTriggerDetection();

            /// Returns a brief description of the block
            std::string description() const override {return    ""
                                                                "   - Inputs: \n"
                                                                "   - Outputs: \n";};

            virtual /// Get custom view widget to be display in the graph
            QWidget * customWidget() override;
        private:
            bool idle_ = true;

            QComboBox * typeSelection_;
            QSpinBox * labelSelection_;

        };
    }
}

#endif