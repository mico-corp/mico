//---------------------------------------------------------------------------------------------------------------------
//  FLOW
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

#include <string>

namespace flow{

    /// sprintf substitution order:
    /// 1) name of class
    /// 2) friendly name of class
    /// 3) name of class
    /// 4) Description of class
    static const std::string templateHeader = 
        "//---------------------------------------------------------------------------------------------------------------------\n"
        "//  Custom MICO plugin\n"
        "//---------------------------------------------------------------------------------------------------------------------\n"
        "//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com\n"
        "//---------------------------------------------------------------------------------------------------------------------\n"
        "//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software\n"
        "//  and associated documentation files (the \"Software\"), to deal in the Software without restriction,\n"
        "//  including without limitation the rights to use, copy, modify, merge, publish, distribute,\n"
        "//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is\n"
        "//  furnished to do so, subject to the following conditions:\n"
        "//\n"
        "//  The above copyright notice and this permission notice shall be included in all copies or substantial\n"
        "//  portions of the Software.\n"
        "//\n"
        "//  THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING\n"
        "//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND\n"
        "//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES\n"
        "//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.\n"
        "//---------------------------------------------------------------------------------------------------------------------\n"
        "\n"
        "\n"
        "\n"
        "#ifndef MICO_FLOW_BLOCKS_ARITHMETICBLOCKS_H_\n"
        "#define MICO_FLOW_BLOCKS_ARITHMETICBLOCKS_H_\n"
        "\n"
        "#include <flow/Block.h>\n"
        "class %1: public flow::Block{\n"
        "public:\n"
        "    /// Get name of block\n"
        "    virtual std::string name() const override {return \"%1\";}\n"
        "\n"
        "   /// Base constructor\n"
        "    %1();\n"
        "\n"
        "    /// Returns a brief description of the block\n"
        "    std::string description() const override {return    \"%2\";};\n"
        "\n"
        "};\n";

}