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

#include <mico/dvs/flow/BlockCornerDetector.h>

namespace mico {
namespace dvs {

BlockCornerDetector::BlockCornerDetector() {
  detectorList_ = {new FastDetector, new HarrisDetector};

  initVisualization();

  createPipe<PolarityPacket>("corners");

  createPolicy({flow::makeInput<PolarityPacket>("events")});

  registerCallback({"events"}, &BlockCornerDetector::processEvents, this);
}

void BlockCornerDetector::processEvents(PolarityPacket _events) {

  auto events = _events;
  FastDetector detector;
  detectorGuard_.lock();
  auto cornerEvents = std::make_shared<libcaer::events::PolarityEventPacket>(
      events->size(), events->getEventSource(), events->getEventSize());

  int idxCorner = 0;
  for (libcaer::events::EventPacket::size_type i = 0; i < events->size(); i++) {
    // Get full timestamp and addresses of first event.
    const libcaer::events::PolarityEvent &polEvent = (*events)[i];

    int32_t ts = polEvent.getTimestamp();
    uint16_t x = polEvent.getX();
    uint16_t y = polEvent.getY();
    bool pol = polEvent.getPolarity();
    dv::Event event(static_cast<int64_t>(ts), static_cast<int16_t>(x),
                    static_cast<int16_t>(y), static_cast<uint8_t>(pol));
    if (polEvent.isValid() && detector.isFeature(event)) {
      cornerEvents->getEvent(idxCorner) = polEvent;
      idxCorner++;
      cornerEvents->getHeaderPointer()->eventNumber++;
      // std::cout << i << "/" << idxCorner << "/" <<
      // cornerEvents->getEventNumber() << std::endl;
    }
  }
  detectorGuard_.unlock();
  if (cornerEvents->getEventNumber() > 0) {
    getPipe("corners")->flush(
        std::const_pointer_cast<const libcaer::events::PolarityEventPacket>(
            cornerEvents));
  }
}

void BlockCornerDetector::initVisualization() {
  visualContainer_ = new QGroupBox();
  mainLayout_ = new QVBoxLayout();
  visualContainer_->setLayout(mainLayout_);

  detectorSelector_ = new QComboBox();
  mainLayout_->addWidget(detectorSelector_);

  for (auto &detector : detectorList()) {
    detectorSelector_->addItem(detector->name().c_str());
  }

  QWidget::connect(detectorSelector_,
                   QOverload<int>::of(&QComboBox::currentIndexChanged),
                   [this](int _n) { this->changeDetector(_n); });
  currentDetector_ = detectorList_[0];
  detectorSelector_->setCurrentIndex(0);
}

std::vector<Detector *> BlockCornerDetector::detectorList() {
  return detectorList_;
}

void BlockCornerDetector::changeDetector(int _index) {
  detectorGuard_.lock();

  if (currentDetector_->customWidget()) {
    currentDetector_->customWidget()->setVisible(false);
    mainLayout_->removeWidget(currentDetector_->customWidget());
  }
  currentDetector_ = detectorList_[_index];
  auto widget = currentDetector_->customWidget();
  if (widget) {
    widget->setVisible(true);
    mainLayout_->addWidget(widget);
  }

  detectorGuard_.unlock();
}

bool BlockCornerDetector::configure(
    std::vector<flow::ConfigParameterDef> _params) {
  int imageWidth = 640, imageHeight = 480;
  if (auto param = getParamByName(_params, "image_width"); param)
    imageWidth = param.value().asInteger();

  if (auto param = getParamByName(_params, "image_height"); param)
    imageHeight = param.value().asInteger();

  detectorGuard_.lock();
  for (auto &detector : detectorList_) {
    detector->setWidth(imageWidth);
    detector->setHeight(imageHeight);
    detector->update();
  }
  detectorGuard_.unlock();

  return true;
}

std::vector<flow::ConfigParameterDef> BlockCornerDetector::parameters() {
  return {
      {"image_width", flow::ConfigParameterDef::eParameterType::INTEGER, 346},
      {"image_height", flow::ConfigParameterDef::eParameterType::INTEGER, 260}};
}
} // namespace dvs
} // namespace mico
