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

#include <mico/dvs/flow/BlockEventsToCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace mico{

namespace dvs{

    BlockEventsToCloud::BlockEventsToCloud(){

        createPipe<pcl::PointXYZRGBNormal>("cloud");
        
        createPolicy({flow::makeInput<dv::EventStore>("events")});
        registerCallback({"events"},
		&BlockEventsToCloud::transformEvents, this
        );
    }

    void BlockEventsToCloud::transformEvents(dv::EventStore _events){
            auto events = _events;
                
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr eventsCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
            if( obtainPointCloudFromEvents(events,eventsCloud) ){

                if(getPipe("cloud")->registrations() !=0 )
                    getPipe("cloud")->flush(eventsCloud); 
            }

        }

    bool BlockEventsToCloud::obtainPointCloudFromEvents(dv::EventStore _events , pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &_cloud){
        if (_events.size() == 0){
            std::cout << " empty events store \n";
            return false;
        }

        // 666 set Z as first ev timestamp - actual event
        bool firstSaved = false;
        int64_t firstTimestamp = 0;
        for (const dv::Event &ev : _events){
            if (!firstSaved){
                firstTimestamp = ev.timestamp();
                firstSaved  = true;
            }
            pcl::PointXYZRGBNormal pt;
            pt.x = static_cast<float>(ev.x());
            pt.y = static_cast<float>(ev.y());
            pt.z = static_cast<float>((ev.timestamp() - firstTimestamp) * 0.001);
            
            pt.b = 0;
            if (ev.polarity()){
                pt.g = 255;
                pt.r = 0;
            }else{
                pt.g = 0;
                pt.r = 255;
            }

            std::cout << "pz " << pt.z << std::endl;

            _cloud->points.push_back(pt);
        }

        return true;
    }
}

    
}
