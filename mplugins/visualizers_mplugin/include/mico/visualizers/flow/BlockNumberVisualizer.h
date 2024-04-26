//-----------------------------------------------------------------------------
//  Visualizers MICO plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//-----------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to
//  deal in the Software without restriction, including without limitation the
//  rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
//  sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
//  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
//  IN THE SOFTWARE.
//-----------------------------------------------------------------------------

#ifndef MICO_FLOW_STREAMERS_BLOCKS_BLOCKNUMBERVISUALIZER_H_
#define MICO_FLOW_STREAMERS_BLOCKS_BLOCKNUMBERVISUALIZER_H_

#include <flow/Block.h>

class QLabel;
class QTimer;

namespace mico {

namespace visualizer {

/// Mico block for visualizing streams of numbers.
/// @ingroup  mico_visualizer
///
/// @image html blocks/visualizers/visualizers_block_number.png width=480px
///
/// __Inputs__:
///     * Number: number to be displayed
///
class BlockNumberVisualizer : public flow::Block {
public:
  /// Get name of block
  std::string name() const override { return "Number Visualizer"; }

  BlockNumberVisualizer();

  ~BlockNumberVisualizer();

  /// Returns a brief description of the block
  std::string description() const override {
    return "Simple number visualizer block.        \n"
           "   - Inputs: Number to be displayed    \n";
  };

  /// Return if the block is configurable.
  bool isConfigurable() override { return false; };

  /// Get custom view widget to be display in the graph
  QWidget *customWidget();

private:
  float number_ = 0;

  QLabel *textDisplay_;
  QTimer *refreshTimer_;
};
} // namespace visualizer
} // namespace mico

#endif