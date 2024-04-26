//-----------------------------------------------------------------------------
//  FLOW
//-----------------------------------------------------------------------------
//  Copyright 2020 - Pablo Ramon Soria (a.k.a. Bardo91)
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

#ifndef FLOW_TESTS_INPUTGENERATORS_H_
#define FLOW_TESTS_INPUTGENERATORS_H_

#include <vector>

// This class infer the type of an input by its name. It is heavily hardcoded...
// There should be a way to make extensible to any class. Create a task to do it
// later 666 TODO
class InputGenerator {
public:
  static InputGenerator *create(const std::string &type_);

  virtual boost::any defaultContructible() = 0;
  virtual boost::any randomInput() = 0;
};

class InputGeneratorBool : public InputGenerator {
public:
  virtual boost::any defaultContructible() override { return false; }

  virtual boost::any randomInput() override { return rand() > RAND_MAX / 2; }
};

class InputGeneratorInt : public InputGenerator {
public:
  virtual boost::any defaultContructible() override { return 0; }

  virtual boost::any randomInput() override { return rand(); }
};

class InputGeneratorFloat : public InputGenerator {
public:
  virtual boost::any defaultContructible() override { return 0.0f; }

  virtual boost::any randomInput() override { return float(rand()) / rand(); }
};

class InputGeneratorVectorInt : public InputGenerator {
public:
  virtual boost::any defaultContructible() override {
    return std::vector<int>{};
  }

  virtual boost::any randomInput() override {
    std::vector<int> v;
    size_t n = (float(rand()) / RAND_MAX) * 1000;
    v.resize(n);
    for (unsigned i = 0; i < n; i++) {
      v[i] = rand();
    }
    return v;
  }
};

class InputGeneratorVectorFloat : public InputGenerator {
public:
  virtual boost::any defaultContructible() override {
    return std::vector<float>{};
  }

  virtual boost::any randomInput() override {
    std::vector<float> v;
    size_t n = (float(rand()) / RAND_MAX) * 1000;
    v.resize(n);
    for (unsigned i = 0; i < n; i++) {
      v[i] = float(rand()) / rand();
    }
    return v;
  }
};

InputGenerator *InputGenerator::create(const std::string &_type) {
  if (_type == typeid(bool).name()) {
    return new InputGeneratorBool();
  } else if (_type == typeid(int).name()) {
    return new InputGeneratorInt();
  } else if (_type == typeid(float).name()) {
    return new InputGeneratorFloat();
  } else if (_type == typeid(std::vector<int>).name()) {
    return new InputGeneratorVectorInt();
  } else if (_type == typeid(std::vector<float>).name()) {
    return new InputGeneratorVectorFloat();
  } else { // type not implemented
    return nullptr;
  }
}

#endif