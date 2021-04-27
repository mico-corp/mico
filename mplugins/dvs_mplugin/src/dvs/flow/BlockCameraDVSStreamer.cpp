//---------------------------------------------------------------------------------------------------------------------
//  MICO DVS plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova) marrcogrova@gmail.com 
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

#include <mico/dvs/flow/BlockCameraDVSStreamer.h>
#include <dv-sdk/processing/core.hpp>
#include <mico/dvs/Common.h>

#include <QSpinBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>

namespace mico{

namespace dvs{
    BlockCameraDVSStreamer::BlockCameraDVSStreamer(){    
        createPipe<PolarityPacket>("events");
        createPipe<int>("events-size");
    }

    BlockCameraDVSStreamer::~BlockCameraDVSStreamer(){
    	if(davisHandle_)
            davisHandle_->dataStop();
    }

    bool BlockCameraDVSStreamer::configure(std::vector<flow::ConfigParameterDef> _params) {
        if(isRunningLoop()) // Cant configure if already running.
            return false;   

        int id=1;
        if (auto param = getParamByName(_params, "dataset_path"); param)
            id = param.value().asInteger();

        davisHandle_ = new libcaer::devices::davis(id);
        
        // Let's take a look at the information we have on the device.
        struct caer_davis_info davis_info = davisHandle_->infoGet();

        printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", davis_info.deviceString,
            davis_info.deviceID, davis_info.deviceIsMaster, davis_info.dvsSizeX, davis_info.dvsSizeY,
            davis_info.logicVersion);
            
        // Send the default configuration before using the device.
        // No configuration is sent automatically!
        davisHandle_->sendDefaultConfig();

        // Tweak some biases, to increase bandwidth in this case.
        struct caer_bias_coarsefine coarseFineBias;

        coarseFineBias.coarseValue        = 2;
        coarseFineBias.fineValue          = 116;
        coarseFineBias.enabled            = true;
        coarseFineBias.sexN               = false;
        coarseFineBias.typeNormal         = true;
        coarseFineBias.currentLevelNormal = true;

        davisHandle_->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias));

        coarseFineBias.coarseValue        = 1;
        coarseFineBias.fineValue          = 33;
        coarseFineBias.enabled            = true;
        coarseFineBias.sexN               = false;
        coarseFineBias.typeNormal         = true;
        coarseFineBias.currentLevelNormal = true;

        davisHandle_->configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias));

        // Let's verify they really changed!
        uint32_t prBias   = davisHandle_->configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP);
        uint32_t prsfBias = davisHandle_->configGet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP);

        printf("New bias values --- PR-coarse: %d, PR-fine: %d, PRSF-coarse: %d, PRSF-fine: %d.\n",
            caerBiasCoarseFineParse(prBias).coarseValue, caerBiasCoarseFineParse(prBias).fineValue,
            caerBiasCoarseFineParse(prsfBias).coarseValue, caerBiasCoarseFineParse(prsfBias).fineValue);

        // Now let's get start getting some data from the device. We just loop in blocking mode,
        // no notification needed regarding new events. The shutdown notification, for example if
        // the device is disconnected, should be listened to.
        // davisHandle_->dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);
        davisHandle_->dataStart(nullptr, nullptr, nullptr, nullptr, nullptr);

        // Let's turn on blocking data-get mode to avoid wasting resources.
        davisHandle_->configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

        davisHandle_->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_GLOBAL_SHUTTER, false);
        davisHandle_->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_AUTOEXPOSURE, false);
        davisHandle_->configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, 4200);

        
        return true;
    }

    std::vector<flow::ConfigParameterDef> BlockCameraDVSStreamer::parameters(){
        return {
            {"id", flow::ConfigParameterDef::eParameterType::INTEGER, 1}
        };
    }

    QWidget * BlockCameraDVSStreamer::customWidget(){
        QGroupBox *box = new QGroupBox;
        QHBoxLayout *layout = new QHBoxLayout;
        box->setLayout(layout);
        QLabel *label = new QLabel("events Hz");
        layout->addWidget(label);

        QSpinBox *rateController = new QSpinBox;
        rateController->setMinimum(1);
        rateController->setMaximum(100000);
        rateController->setValue(int(eventRate_));
        layout->addWidget(rateController);

        QWidget::connect(rateController, QOverload<int>::of(&QSpinBox::valueChanged), [this](int _val){
            eventRate_ = _val;
        });

        return box;
    }


    void BlockCameraDVSStreamer::loopCallback() {
        while(isRunningLoop()){
            std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer = davisHandle_->dataGet();
            if (packetContainer == nullptr) {
                continue; // Skip if nothing there.
            }

            //printf("\nGot event container with %d packets (allocated).\n", packetContainer->size());

            for (auto &packet : *packetContainer) {
                if (packet == nullptr) {
                    //printf("Packet is empty (not present).\n");
                    continue; // Skip if nothing there.
                }

                //printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(), packet->getEventCapacity());

                if(getPipe("events")->registrations() !=0 ){
                    if (packet->getEventType() == POLARITY_EVENT) {
                        std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                            = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);

                        // dv::EventStore rawEvents;
                        // for(unsigned i = 0; i < polarity->size(); i++){
                        //     // Get full timestamp and addresses of first event.
                        //     const libcaer::events::PolarityEvent &firstEvent = (*polarity)[i];

                        //     int32_t ts = firstEvent.getTimestamp();
                        //     uint16_t x = firstEvent.getX();
                        //     uint16_t y = firstEvent.getY();
                        //     bool pol   = firstEvent.getPolarity();

                        //     //printf("First polarity event - ts: %d, x: %d, y: %d, pol: %d.\n", ts, x, y, pol);
                        //     dv::Event event(static_cast<int64_t>(ts) , static_cast<int16_t>(x) , static_cast<int16_t>(y) , static_cast<uint8_t>(pol));

                        //     rawEvents.add(event);
                        // }

                        getPipe("events")->flush(polarity);    
                    }

                }
                /*
                if (packet->getEventType() == FRAME_EVENT) {
                    std::shared_ptr<const libcaer::events::FrameEventPacket> frame = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);

                    // Get full timestamp, and sum all pixels of first frame event.
                    const libcaer::events::FrameEvent &firstEvent = (*frame)[0];

                    int32_t ts   = firstEvent.getTimestamp();
                    uint64_t sum = 0;

                    for (int32_t y = 0; y < firstEvent.getLengthY(); y++) {
                        for (int32_t x = 0; x < firstEvent.getLengthX(); x++) {
                            sum += firstEvent.getPixel(x, y);
                        }
                    }

                    printf("First frame event - ts: %d, sum: %" PRIu64 ".\n", ts, sum);
                }*/
            }

        //     if(getPipe("events-size")->registrations() !=0 ){
        //         dv::EventStore store;
        //         streamer_->events(store);
        //         std::cout <<static_cast<int>(store.size()) <<std::endl;
        //         getPipe("events-size")->flush(static_cast<int>(store.size()));
        //     }    
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

}
}