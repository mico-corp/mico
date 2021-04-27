//---------------------------------------------------------------------------------------------------------------------
//  Vertical Engineering Solutions
//---------------------------------------------------------------------------------------------------------------------
// 
//  Copyright 2020 Vertical Engineering Solutions  - All Rights Reserved
// 
//  Unauthorized copying of this file, via any medium is strictly prohibited Proprietary and confidential.
// 
//  All information contained herein is, and remains the property of Vertical Engineering Solutions.  The 
//  intellectual and technical concepts contained herein are proprietary to Vertical Engineering Solutions 
//  and its suppliers and may be covered by UE and Foreign Patents, patents in process, and are protected 
//  by trade secret or copyright law. Dissemination of this information or reproduction of this material is 
//  strictly forbidden unless prior written permission is obtained from Vertical Engineering Solutions.
//
//---------------------------------------------------------------------------------------------------------------------
//
//  Maintainer: pramon@vengineerings.com
//
//---------------------------------------------------------------------------------------------------------------------

#include <mico/robotics/PID.h>
#include <algorithm>
#include <thread>
#include <chrono>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>

#include <cmath>

namespace mico{

    namespace robotics{
        PID::PID(float _kp, float _ki, float _kd, float _fc, float _minSat, float _maxSat) {
            kp_ = _kp;
            ki_ = _ki;
            kd_ = _kd;
            fc_ = _fc;
            assert(_minSat <= _maxSat);
            minSat_ = _minSat;
            maxSat_ = _maxSat;
            clear();
        }

        /*void PID::setAntiWindup(AntiWindupMethod _antiWindup, std::vector<float> _params){
            
        }*/

        void PID::clear(){
            lastError_ = 0; 
            lastResult_ = 0; 
            accumErr_ = 0; 
            accumDeriv_ = 0;
        }

        void PID::overrideAccumulative(float _accumulative, bool _isOutputScale){
            if(!std::isnan(_accumulative)){
                if(_isOutputScale){
                    if(ki_ != 0.0f){
                        accumErr_ = _accumulative/ki_;
                    }
                }else{
                    accumErr_ = _accumulative;
                }
            }
        }

        void PID::reference(float _ref, float _time, bool _reset) { 
            reference_ = _ref; 
            targetReference_ = _ref;
            // if(_time == 0){
            //     reference_ = _ref; 
            //     targetReference_ = _ref;
            // }else{
            //     targetReference_ = _ref;
            //     slope_ = (targetReference_ - reference_)/_time;
            // }

            if(_reset){
                clear();
            }
        }


        float PID::update(float _val, float _incT) {

            float dt = _incT; // 666 input arg?
            float err = reference_ - _val;
            
            float up = kp_ * err;

            accumErr_ += err * dt;
            float ui = ki_ * accumErr_;

            float ud = ((kd_ * err) - accumDeriv_) * fc_ ;
            accumDeriv_ += ud * dt;

            // Compute PID
            float unsaturated =   up + ui + ud;

            lastError_ = err;

            // Saturate signal
            float saturated = std::min(std::max(unsaturated, minSat_), maxSat_);
            lastResult_ = saturated;
            
            return lastResult_;

        }
    }
}

