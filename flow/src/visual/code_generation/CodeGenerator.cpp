
//---------------------------------------------------------------------------------------------------------------------
//  flow
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


#ifdef HAS_QTNODEEDITOR

#include <flow/visual/code_generation/CodeGenerator.h>
#include <iostream>
#include <memory>

#include <QJsonArray>
#include <QStringList>

#include <boost/core/demangle.hpp>

#ifdef _WIN32
    #include <windows.h> 
#endif


namespace flow{
    // Initialize dictionary
    void CodeGenerator::parseScene(std::string _cppFile, QJsonObject const &_scene, const std::vector<std::string> &_customIncludes){
        //std::ofstream genMain(_cppFile);
    
        //// Write preamble
        //writeInit(genMain, _customIncludes);
        //
        //genMain << "" << std::endl;
        //genMain << "\t// Creating blocks" << std::endl;
        //genMain << "" << std::endl;
        //// Init Nodes
        //std::vector<flow::ConfigParameterDef> id2Name;
        //std::unordered_map<std::string, QJsonArray> id2OutTags;
        //QJsonArray nodesJsonArray = _scene["nodes"].toArray();
        //int nodeCounter = 0;
        //for (QJsonValueRef node : nodesJsonArray){
        //    std::string id = node.toObject()["id"].toString().toStdString();
        //    std::string classType = node.toObject()["model"].toObject()["class"].toString().toStdString();
        //    classType = boost::core::demangle(classType.c_str());
        //    classType = classType.substr(0, classType.size()-1);

        //    id2Name[id] = "node_" + std::to_string(nodeCounter);
        //    id2OutTags[id] = node.toObject()["model"].toObject()["out_tags"].toArray();
        //    nodeCounter++;

        //    genMain << "\t" << classType << " " << id2Name[id] << ";" << std::endl;


        //    QJsonObject blockParams = node.toObject()["model"].toObject()["params"].toObject();
        //    unsigned int nKeys = blockParams.keys().size();
        //    if(nKeys > 0){
        //        genMain << "\t" << id2Name[id] << ".configure({"<<std::endl;
        //        for(unsigned i = 0; i < nKeys; i++){
        //            QString key = blockParams.keys()[i];
        //            genMain << "\t\t" << "{\"" << key.toStdString() << "\",\"" << blockParams[key].toString().toStdString() << "\"}";
        //            if(i < (nKeys-1)){
        //                genMain << ",";
        //            }
        //            genMain << std::endl;

        //        }
        //        genMain << "\t" << "});" <<std::endl << std::endl;
        //    }
        //}

        //genMain << "" << std::endl;
        //genMain << "\t// Connecting blocks" << std::endl;
        //genMain << "" << std::endl;

        //// Connect nodes
        //QJsonArray connectionJsonArray = _scene["connections"].toArray();
        //for (QJsonValueRef connectionRef : connectionJsonArray) {
        //    auto connection = connectionRef.toObject();
        //    auto inId = connection["in_id"].toString().toStdString();
        //      unsigned inIdx = connection["in_index"].toInt();
        //    auto outId = connection["out_id"].toString().toStdString();
        //    unsigned outIdx = connection["out_index"].toInt();
        //    
        //    
        //    std::string tag = id2OutTags[outId][outIdx].toString().toStdString();

        //    genMain << "\t" << id2Name[outId] << ".connect(\"" << tag << "\", " << id2Name[inId] << ");\n";
        //}

        //// Start autoloop
        //genMain << "" << std::endl;
        //genMain << "\t// Starting autoloopable blocks" << std::endl;
        //genMain << "" << std::endl;


        //for (QJsonValueRef node : nodesJsonArray){
        //    std::string id = node.toObject()["id"].toString().toStdString();
        //    bool hasAutoloop = node.toObject()["model"].toObject()["autoloop"].toBool();   

        //    if(hasAutoloop){
        //        genMain << "\t" << id2Name[id] << ".start();"<<std::endl;
        //    }

        //}
        //// Write end
        //writeEnd(genMain);
    }

    void CodeGenerator::generateCmake(  std::string _cmakeFilePath, 
                                        std::string _cppName, 
                                        const std::vector<std::string> &_customFinds, 
                                        const std::vector<std::string> &_customLinks){
        std::ofstream cmakeFile(_cmakeFilePath);

        auto exePath = _cppName.substr(0, _cppName.size()-4); // Remove cpp
        auto lastBar = exePath.find_last_of('/');
        exePath = exePath.substr(lastBar+1, exePath.size());   // Get just name

        cmakeFile << "cmake_minimum_required (VERSION 3.12 FATAL_ERROR)" << std::endl;
        cmakeFile << "project(mico VERSION 1.0 LANGUAGES C CXX)" << std::endl;
        
        for(auto &cFinds: _customFinds){
            cmakeFile << "find_package(" + cFinds + ")" << std::endl;
        }


        cmakeFile << "add_executable(" +exePath + " " +_cppName+")" << std::endl;

        for(auto &cLinks: _customLinks){
            cmakeFile <<  "target_link_libraries(" << exePath << " LINK_PRIVATE "<< cLinks <<")" << std::endl;
        }

    }

    void CodeGenerator::compile(std::string _cppFolder){
        std::string cmd = "cd "+ _cppFolder +"&& mkdir -p build && cd build && cmake .. && make";
        system(cmd.c_str());

    }


    void CodeGenerator::writeInit(std::ofstream &_file, const std::vector<std::string> &_customIncludes){

        _file <<    "//---------------------------------------------------------------------------------------------------------------------"<< std::endl;
        _file <<    "//  flow"<< std::endl;
        _file <<    "//---------------------------------------------------------------------------------------------------------------------"<< std::endl;
        _file <<    "//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com"<< std::endl;
        _file <<    "//---------------------------------------------------------------------------------------------------------------------"<< std::endl;
        _file <<    "//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software"<< std::endl;
        _file <<    "//  and associated documentation files (the \"Software\"), to deal in the Software without restriction,"<< std::endl;
        _file <<    "//  including without limitation the rights to use, copy, modify, merge, publish, distribute,"<< std::endl;
        _file <<    "//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is"<< std::endl;
        _file <<    "//  furnished to do so, subject to the following conditions:"<< std::endl;
        _file <<    "//"<< std::endl;
        _file <<    "//  The above copyright notice and this permission notice shall be included in all copies or substantial"<< std::endl;
        _file <<    "//  portions of the Software."<< std::endl;
        _file <<    "//"<< std::endl;
        _file <<    "//  THE SOFTWARE IS PROVIDED \"AS IS\", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING"<< std::endl;
        _file <<    "//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND"<< std::endl;
        _file <<    "//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES"<< std::endl;
        _file <<    "//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN"<< std::endl;
        _file <<    "//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE."<< std::endl;
        _file <<    "//---------------------------------------------------------------------------------------------------------------------"<< std::endl;

        _file <<    ""                                      << std::endl;
        _file <<    "// FLOW AUTO-GENERATED FILE"           << std::endl;
        _file <<    ""                                      << std::endl;
        
        for(auto &cIncludes: _customIncludes){
            _file <<    "#include <" << cIncludes << ">"      << std::endl;
        }

        _file <<    "#include <csignal>"                    << std::endl;
        _file <<    ""                                      << std::endl;
        _file <<    "bool run = true;"                      << std::endl;
        _file <<    "void signal_handler(int signal) {"     << std::endl;
        _file <<    "    if(signal == SIGINT){"             << std::endl;
        _file <<    "        run = false;"                  << std::endl;
        _file <<    "    }"                                 << std::endl;
        _file <<    "}"                                     << std::endl;
        _file <<    ""                                      << std::endl;
        _file <<    "using namespace flow;"                 << std::endl;
        _file <<    ""                                      << std::endl;
        _file <<    "int main(int _argc, char ** _argv){"   << std::endl;
        _file <<    ""                                      << std::endl;
    }

    void CodeGenerator::writeEnd(std::ofstream &_file){
        _file << "\twhile(run) {" << std::endl;
        _file << "\t\tstd::this_thread::sleep_for(std::chrono::milliseconds(500));" << std::endl;
        _file << "\t}" << std::endl;
        _file << "" << std::endl;
        _file << "}" << std::endl;
    }



    std::string CodeGenerator::demangleClassType(const char* mangled) {
        #ifdef linux
            int status;
            std::unique_ptr<char[], void (*)(void*)> result(abi::__cxa_demangle(mangled, 0, 0, &status), std::free);
            std::string str = result.get() ? std::string(result.get()) : "error occurred";
            return str;
        #endif
        #ifdef _WIN32
            /*TCHAR szUndecorateName[256];
            memset(szUndecorateName, 0, 256);
            UnDecorateSymbolName(mangled, szUndecorateName, 256, 0);
            std::string str(szUndecorateName);
            return str;*/
            assert(false);
            return "";
        #endif
    }

}

#endif