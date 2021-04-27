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


#ifndef AEROXSUITE_CONTROL_PID_H_
#define AEROXSUITE_CONTROL_PID_H_


#include <limits>
#include <thread>
#include <functional>

#include <cmath>

namespace mico{
    namespace robotics{
        /// Just another implementation of a PID controller.
        /// @ingroup  mico_robotics
        class PID {
        public:
            struct PIDParams{
                float kp, ki, kd, sat, wind, fc;
            };

            enum class AntiWindupMethod { None, Saturation, BackCalculation, Clamping};

            PID(float _kp, float _ki, float _kd, float fc_ = 30, 
                float _minSat = -std::numeric_limits<float>::max(),
                float _maxSat = std::numeric_limits<float>::max() );
            
            void setAntiWindup(AntiWindupMethod _antiWindup, std::vector<float> _params = {});


            float update(float _val, float _incT);
            
            void clear();

            /// Override internal value of accumulative term (Integral component). 
            /// \param _accumulative: magnitude to be written. If negative, value is ignored.
            /// \param _isOutputScale: if true, the _accumulative term will be divided by Ki to scale it. If false, the value is set as given.
            void overrideAccumulative(float _accumulative, bool _isOutputScale);

            float reference() { return reference_; }
            void reference(float _ref, float time = 0, bool _reset = true);

            float kp() const { return kp_; }
            float ki() const { return ki_; }
            float kd() const { return kd_; }
            float fc() const { return fc_; }
        
            void kp(float _kp) { kp_ = _kp; }
            void ki(float _ki) { 
                if(_ki != 0.0f){
                    // Scale accumulated error to prevent abrupt changes in integral component of control signal
                    accumErr_ *=ki_/_ki;    
                }
                ki_ = _ki; 
            }
            void kd(float _kd) { kd_ = _kd; }
            void fc(float _fc) { fc_ = _fc; }
        
            void setSaturations(float _min, float _max) { minSat_ = _min; maxSat_ = _max; }
            void getSaturations(float &_min, float &_max) { _min = minSat_; _max = maxSat_; }
        
        private:
            float reference_ = 0;
            float targetReference_ = 0;
            float slope_ = 0;

            float kp_ = 0, ki_ = 0, kd_ = 0, fc_ = 0;;
            float minSat_ = 0, maxSat_ = 0;
            float lastResult_ = 0; 
            float lastError_ = 0;
            float accumErr_ = 0;
            float accumDeriv_ = 0;
            
            // Params related with antiwindups
            // Saturation
            //float minWindup_ = 0, maxWindup_ = 0;
            // BackCalculation
            float lastBackCalculation_ = 0;
            float backCalculationCte_ = 1;
            // Clamping
            bool clampFactor_ = 1;

        };
    }
}

#endif
