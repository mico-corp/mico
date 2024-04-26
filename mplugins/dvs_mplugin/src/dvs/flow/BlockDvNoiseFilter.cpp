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

#include <mico/dvs/flow/BlockNoiseFilter.h>

#include <mico/dvs/Common.h>
namespace mico {

namespace dvs {

BlockNoiseFilter::BlockNoiseFilter() {

  noiseFilter_ = caerFilterDVSNoiseInitialize(346, 260);

  // Handle constructor failure.
  if (noiseFilter_ == nullptr) {
    std::string exc =
        "Failed to initialize DVS Noise filter, sizeX=" + std::to_string(346) +
        ", sizeY=" + std::to_string(260) + ".";
    throw std::runtime_error(exc);
  }

  // Use stateless lambda for shared_ptr custom deleter.
  auto deleteDeviceHandle = [](caerFilterDVSNoise fh) {
    // Run destructor, free all memory.
    // Never fails in current implementation.
    caerFilterDVSNoiseDestroy(fh);
  };

  noiseFilterHandle_ = std::shared_ptr<struct caer_filter_dvs_noise>(
      noiseFilter_, deleteDeviceHandle);

  createPipe<PolarityPacket>("events");

  createPolicy({flow::makeInput<PolarityPacket>("events")});
  registerCallback({"events"}, &BlockNoiseFilter::filterEvents, this);
}

void BlockNoiseFilter::filterEvents(PolarityPacket _events) {
  caerFilterDVSNoiseApply(noiseFilterHandle_.get(),
                          (caerPolarityEventPacket)_events->getHeaderPointer());
  getPipe("events")->flush(_events);
}

std::vector<flow::ConfigParameterDef> BlockNoiseFilter::parameters() {
  return {
      {"enable_two_levels", flow::ConfigParameterDef::eParameterType::BOOLEAN,
       true},
      {"enable_check_pol", flow::ConfigParameterDef::eParameterType::BOOLEAN,
       true},
      {"support_min", flow::ConfigParameterDef::eParameterType::INTEGER, 2},
      {"support_max", flow::ConfigParameterDef::eParameterType::INTEGER, 8},
      {"time_window", flow::ConfigParameterDef::eParameterType::INTEGER, 2000}};
}

bool BlockNoiseFilter::configure(
    std::vector<flow::ConfigParameterDef> _params) {

  if (auto param = getParamByName(_params, "enable_two_levels"); param)
    twoLevels_ = param.value().asBool();

  if (auto param = getParamByName(_params, "enable_check_pol"); param)
    checkPol_ = param.value().asBool();

  if (auto param = getParamByName(_params, "support_min"); param)
    supportMin_ = param.value().asInteger();

  if (auto param = getParamByName(_params, "support_max"); param)
    supportMax_ = param.value().asInteger();

  if (auto param = getParamByName(_params, "time_window"); param)
    actTime_ = param.value().asInteger();

  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TWO_LEVELS,
                              twoLevels_);
  caerFilterDVSNoiseConfigSet(
      noiseFilterHandle_.get(),
      CAER_FILTER_DVS_BACKGROUND_ACTIVITY_CHECK_POLARITY, checkPol_);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MIN,
                              supportMin_);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MAX,
                              supportMax_);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME,
                              actTime_);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE, true);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME, 200);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE, true);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_HOTPIXEL_ENABLE, true);
  caerFilterDVSNoiseConfigSet(noiseFilterHandle_.get(),
                              CAER_FILTER_DVS_HOTPIXEL_LEARN, true);

  return true;
}

} // namespace dvs
} // namespace mico
