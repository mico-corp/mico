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

    static const std::string templatePolicy = 
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
        "\n";



    /// Arguments substitution:
    ///  1) PAth to header
    static const std::string templateBasicIncludesSources = 
        "#include <mico/custom/flow/%1.h>\n"
        "#include <flow/Outpipe.h>\n"
        "#include <flow/Policy.h>\n";


    /// Arguments substitution:
    /// 2) Name pipe
    /// 1) Type pipe
    static const std::string templatePipeCreation = "    createPipe<%2>(%1);\n";

    /// Arguments substitution:
    /// 2) Input name
    /// 1) Input type
    static const std::string templateInput = "flow::makeInput<%2>(\"%1\")";
    

    /// Arguments substitution:
    /// 1) List of inputs
    static const std::string templatePolicyCreation =  "    createPolicy({%1});\n";

    /// Arguments substitution:
    /// 1) name of class
    static const std::string templateInitConstructor = 
        "%1::%1(){\n";
    
    static const std::string templateEndConstructor = 
        "};\n";

    /// Arguments substitution:
    /// 1) name of input
    /// 2) type of input
    static const std::string templateGetData = "_data.get<%2>(\"%1\");\n";
    
    
    /// Arguments substitution:
    /// 1) name of output
    /// 2) variable to be flushed
    static const std::string templateFlushData = "getPipe(\"%1\")->flush(%2);\n";

    /// Arguments substitution:
    /// 1) inputs
    /// 2) method function
    static const std::string templateCallback = 
    "registerCallback({%1}, \n"
    "                        [&](flow::DataFlow _data){\n"
    "                           %2"
    "                        }\n"
    ");\n";

}