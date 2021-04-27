
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

#include <flow/visual/FlowVisualInterface.h>

#include <nodes/NodeData>
#include <nodes/Node>
#include <nodes/FlowScene>
#include <nodes/FlowView>
#include <nodes/ConnectionStyle>
#include <nodes/FlowViewStyle>

#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QFileDialog>
#include <QtCore/QByteArray>
#include <QtCore/QBuffer>
#include <QtCore/QDataStream>
#include <QtCore/QFile>
#include <QtCore/QJsonDocument>
#include <QtCore/QJsonObject>

#ifdef foreach  // To be able to use Qt and RealSense Device
  #undef foreach
#endif

#include <flow/flow.h>
#include <flow/visual/blocks/FlowVisualBlock.h>
#include <flow/visual/code_generation/CodeGenerator.h>

#ifdef linux
    #include <X11/Xlib.h>   
    #include <dlfcn.h>
#endif

#ifdef _WIN32
    #include <windows.h>
    #include <dlfcn.h>
#endif

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

using QtNodes::FlowView;
using QtNodes::FlowScene;
using QtNodes::ConnectionStyle;

//
//static void setStyle() {
//  ConnectionStyle::setConnectionStyle(
//    R"(
//  {
//    "ConnectionStyle": {
//      "UseDataDefinedColors": true
//    }
//  }
//  )");
//}

namespace boost {
    void throw_exception(std::exception const& e) { // user defined
        throw e;
    }
}

namespace flow{
    int FlowVisualInterface::init(int _argc, char** _argv){
        
    #ifdef linux
        XInitThreads();	
    #endif

        // Parse arguments
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help,h", "produce help message")
            ("mico_graph,g", po::value<std::string>(), "Path to existing mico graph")
        ;

        po::variables_map vm;        
        po::store(po::parse_command_line(_argc, _argv, desc), vm);
        po::notify(vm);    

        if (vm.count("help")) {
            std::cout << desc << "\n";
            return 1;
        }

        // Init Qapp
        kids_app = new QApplication(_argc, _argv);

        QWidget mainWidget;
        auto menuBar    = new QMenuBar();
        auto saveAction = menuBar->addAction("Save");
        auto loadAction = menuBar->addAction("Load");
        auto configureAll = menuBar->addAction("Configure All");
        auto runAll = menuBar->addAction("Run All");
        auto generateCode = menuBar->addAction("Generate Code");

        QVBoxLayout *l = new QVBoxLayout(&mainWidget);
        l->addWidget(menuBar);

        setStyle();

        scene = new FlowScene(     registerDataModels(),
                                        &mainWidget
                                        );

        l->addWidget(new FlowView(scene));
        l->setContentsMargins(0, 0, 0, 0);
        l->setSpacing(0);

        QObject::connect(saveAction, &QAction::triggered, scene, &FlowScene::save);

        QObject::connect(loadAction, &QAction::triggered, scene, &FlowScene::load);


        QObject::connect(configureAll, &QAction::triggered, [&](){
            this->configureAll();
        });

        QObject::connect(runAll, &QAction::triggered, [&](){
            this->runAll();
        });

        QObject::connect(generateCode, &QAction::triggered, [&](){
            QString fileName = QFileDialog::getOpenFileName(nullptr,
                                                "Select scene to save",
                                                QDir::homePath(),
                                                "Flow Scene Files (*.flow)");

            if (!QFileInfo::exists(fileName))
            return;

            QFile file(fileName);

            if (!file.open(QIODevice::ReadOnly))
            return;

            std::string cppFile = fileName.toStdString().substr(0, fileName.size()-4) + "cpp";
            auto lastBar = cppFile.find_last_of('/');
            std::string cppFolder = cppFile.substr(0, lastBar);
            std::string cmakeFilePath = cppFolder + "/CMakeLists.txt";


            QByteArray wholeFile = file.readAll();
            QJsonObject const jsonDocument = QJsonDocument::fromJson(wholeFile).object();
            CodeGenerator::parseScene(cppFile,jsonDocument, customIncludes_);
            CodeGenerator::generateCmake(cmakeFilePath, cppFile, customFinds_, customLinks_);
            CodeGenerator::compile(cppFolder);
        });

        mainWidget.setWindowTitle("Node-based flow editor");
        mainWidget.resize(800, 600);
        
        if (vm.count("mico_graph")) {
            scene->clearScene();
            QString fileName = vm["mico_graph"].as<std::string>().c_str();
            
            if (!QFileInfo::exists(fileName))
                return -1;

            QFile file(fileName);

            if (!file.open(QIODevice::ReadOnly))
                return -1;

            QByteArray wholeFile = file.readAll();

            scene->loadFromMemory(wholeFile);
            this->configureAll();
            this->runAll();
        } else {
            mainWidget.showNormal();
        }

        return kids_app->exec();
    
    }


    void  FlowVisualInterface::quit(){
        if(kids_app != nullptr){
            kids_app->quit();
        }
    }


    void FlowVisualInterface::setNodeRegisterFn(std::function<void(std::shared_ptr<QtNodes::DataModelRegistry> &_registry)> _fn){
        registerFn_ = _fn;
    }

    void FlowVisualInterface::setCodeGeneratorCustoms(   const std::vector<std::string> &_customIncludes, 
                                                        const std::vector<std::string> &_customFinds, 
                                                        const std::vector<std::string> &_customLinks ){

        customIncludes_ = _customIncludes;
        customFinds_ = _customFinds;
        customLinks_ = _customLinks;
    }

    std::shared_ptr<QtNodes::DataModelRegistry> FlowVisualInterface::registerDataModels(){
        auto registry = std::make_shared<QtNodes::DataModelRegistry>();

        loadCustomPlugins(registry);
        if(registerFn_ != nullptr)
            registerFn_(registry);

        return registry;
    }

    struct PathLeafString {
        std::string operator()(const boost::filesystem::directory_entry& entry) const {
            return entry.path().leaf().string();
        }
    };

    void readDirectory(const std::string& name, std::vector<std::string>& v) {
        boost::filesystem::path p(name);
        if(boost::filesystem::exists(p)){
            boost::filesystem::directory_iterator start(p);
            boost::filesystem::directory_iterator end;
            std::transform(start, end, std::back_inserter(v), PathLeafString());
        }
    }

    void FlowVisualInterface::loadCustomPlugins(std::shared_ptr<QtNodes::DataModelRegistry> &_registry){
        // List plugins in default folder
        std::vector<std::string> files;
        #ifdef linux
            std::string userName(getenv("USER"));
            std::string pluginDir = "/home/"+userName+"/.flow/plugins/";
        #endif
        #ifdef _WIN32
            std::string pluginDir = "C:\\.flow\\plugins\\";
        #endif
        readDirectory(pluginDir, files);
        // Iterate over file
        for(auto file:files){
            std::cout << file << std::endl;
            // Load blocks registered
            void *hndl = dlopen((pluginDir+file).c_str(), RTLD_NOW);
            if(hndl == nullptr){
                std::cerr << "ERROR: " <<  dlerror() << std::endl;
                
            }else{
                dlerror();

                typedef PluginNodeCreator* (*Factory)();
                void *mkr = dlsym(hndl, "factory");
                if(mkr == nullptr){
                    std::cout << "Pluging " << file << " does not have factory" << std::endl;
                    continue;
                }

                Factory factory = (Factory) mkr;

                const char *dlsym_error = dlerror();    
                if (dlsym_error) {
                    std::cerr << "Cannot load symbol 'factory': " << dlsym_error <<            '\n';
                }

                PluginNodeCreator* nodeCreator = factory();
                auto listOfNodeCreators = nodeCreator->get();
                for(auto &nodeCreator: listOfNodeCreators){
                    _registry->registerModel<NodeDataModel>(nodeCreator.second,nodeCreator.first.c_str());
                }
            }
        }
        
    }

    void FlowVisualInterface::configureAll() {
        auto nodes = scene->allNodes();

        for (auto node : nodes) {
            NodeDataModel* dataModel = node->nodeDataModel();
            // This conversion is not safe but, all nodes in slam4kids are ConfigurableBlocks
            auto d_ptr = dynamic_cast<ConfigurableBlock*>(dataModel);
            if (d_ptr != nullptr)
                d_ptr->configure();
        }

    }

    void FlowVisualInterface::runAll() {
        auto nodes = scene->allNodes();

        for (auto node : nodes) {
            NodeDataModel* dataModel = node->nodeDataModel();
            // This conversion is not safe but, all nodes in slam4kids are ConfigurableBlocks
            auto d_ptr = dynamic_cast<RunnableBlock*>(dataModel);
            if (d_ptr != nullptr)
                d_ptr->run();
        }
    }


    void FlowVisualInterface::setStyle() {
        QtNodes::FlowViewStyle::setStyle(
            R"(
      {
        "FlowViewStyle": {
          "BackgroundColor": [255, 255, 240],
          "FineGridColor": [245, 245, 230],
          "CoarseGridColor": [235, 235, 220]
        }
      }
      )");

        QtNodes::NodeStyle::setNodeStyle(
                R"(
      {
        "NodeStyle": {
          "NormalBoundaryColor": "darkgray",
          "SelectedBoundaryColor": "deepskyblue",
          "GradientColor0": "mintcream",
          "GradientColor1": "mintcream",
          "GradientColor2": "mintcream",
          "GradientColor3": "mintcream",
          "ShadowColor": [200, 200, 200],
          "FontColor": [10, 10, 10],
          "FontColorFaded": [100, 100, 100],
          "ConnectionPointColor": "white",
          "PenWidth": 2.0,
          "HoveredPenWidth": 2.5,
          "ConnectionPointDiameter": 10.0,
          "Opacity": 1.0
        }
      }
      )");

            ConnectionStyle::setConnectionStyle(
                R"(
      {
        "ConnectionStyle": {
          "ConstructionColor": "gray",
          "NormalColor": "black",
          "SelectedColor": "gray",
          "SelectedHaloColor": "deepskyblue",
          "HoveredColor": "deepskyblue",
          "LineWidth": 3.0,
          "ConstructionLineWidth": 2.0,
          "PointDiameter": 10.0,
          "UseDataDefinedColors": false
        }
      }
      )");
    }


}


#endif