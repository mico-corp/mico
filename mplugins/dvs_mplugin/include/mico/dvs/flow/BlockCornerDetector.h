//-----------------------------------------------------------------------------
//  MICO DVS plugin
//-----------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com
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

#ifndef BLOCK_DVS_CORNERDETECTOR_H_
#define BLOCK_DVS_CORNERDETECTOR_H_

#include <flow/flow.h>

#include <mico/dvs/Common.h>
#include <mico/dvs/corner_detectors/Detector.h>
#include <mico/dvs/corner_detectors/FastDetector.h>
#include <mico/dvs/corner_detectors/HarrisDetector.h>

#include <mutex>

namespace mico {
namespace dvs {

class BlockCornerDetector : public flow::Block {
public:
  std::string name() const override { return "DV Corner Detector"; };
  std::string description() const override {
    return "Flow wrapper of DVS Corners Detector.\n"
           "- FAST\n"
           "- HARRIS\n";
  };

  BlockCornerDetector();

  virtual QWidget *customWidget() { return visualContainer_; }

  /// Configure block with given parameters.
  virtual bool
  configure(std::vector<flow::ConfigParameterDef> _params) override;

  /// Get list of parameters of the block
  std::vector<flow::ConfigParameterDef> parameters() override;

  /// Return if the block is configurable.
  bool isConfigurable() override { return true; };

private:
  void processEvents(PolarityPacket _events);

  void initVisualization();

  std::vector<Detector *> detectorList();

  void changeDetector(int _index);

private:
  std::mutex detectorGuard_;
  std::vector<Detector *> detectorList_;
  Detector *currentDetector_;

  QComboBox *detectorSelector_ = nullptr;
  QGroupBox *visualContainer_ = nullptr;
  QVBoxLayout *mainLayout_ = nullptr;
};
} // namespace dvs
} // namespace mico

#endif
