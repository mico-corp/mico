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

template<typename T_>
inline void MessageDealer::sendTo(int _id, T_ _data){
      Serial.print(_id);
      Serial.print('@');
      Serial.print(_data);
      Serial.print("\r\n");
}


template<>
inline void MessageDealer::sendTo<std::vector<int>>(int _id, std::vector<int> _data){
      Serial.print(_id);
      Serial.print('@');
      for(auto& d: _data){
        Serial.print(d);
        Serial.print(',');
      }
      Serial.print("\r\n");
}


template<>
inline void MessageDealer::sendTo<std::vector<float>>(int _id, std::vector<float> _data){
      Serial.print(_id);
      Serial.print('@');
      for(auto& d: _data){
        Serial.print(d);
        Serial.print(',');
      }
      Serial.print("\r\n");
}


template<>
inline bool MessageDealer::readFrom<int>(int _id, int &_data){
  if(data_[_id].size() != 0){
    String data = data_[_id].front();
    _data = atoi(data.c_str());
    data_[_id].pop();
    return true;
  }else{
    return false;
  }
}

template<>
inline bool MessageDealer::readFrom<float>(int _id, float &_data){
  if(data_[_id].size() != 0){
    String data = data_[_id].front();
    _data = atof(data.c_str());
    data_[_id].pop();
    return true;
  }else{
    return false;
  }
}

template<>
inline bool MessageDealer::readFrom<std::vector<int>>(int _id, std::vector<int> &_data){
  if(data_[_id].size() != 0){
    String raw = data_[_id].front();
    data_[_id].pop();
    _data.clear();
    
    int i0 = 0;
    int i1 = raw.indexOf(',');
    while(i0 != -1 && i1!=-1){
      String substr = raw.substring(i0,i1);
      _data.push_back(atoi(substr.c_str()));
      i0 = i1+1;
      i1 = raw.indexOf(',', i0);
    }
    return _data.size() != 0;
  }else{
    return false;
  }
}

template<>
inline bool MessageDealer::readFrom<std::vector<float>>(int _id, std::vector<float> &_data){
  if(data_[_id].size() != 0){
    String raw = data_[_id].front();
    data_[_id].pop();
    _data.clear();
    
    int i0 = 0;
    int i1 = raw.indexOf(',');
    while(i0 != -1 && i1!=-1){
      String substr = raw.substring(i0,i1);
      _data.push_back(atof(substr.c_str()));
      i0 = i1+1;
      i1 = raw.indexOf(',', i0);
    }
    return _data.size() != 0;
  }else{
    return false;
  }
}
extern MessageDealer MicoChannel;
