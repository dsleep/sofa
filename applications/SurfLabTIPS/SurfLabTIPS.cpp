#include "Visualizer.hpp"
#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
#include <QIcon>
#include <QProgressDialog>
#include <sofa/helper/system/PluginManager.h>
#include <QDir>

int openWindow(std::string argFile) {
    // Loading the scene
    sofa::helper::system::DataRepository.addFirstPath("/home/saleh/projects/sofa/examples");
    sofa::helper::system::DataRepository.addFirstPath("/home/saleh/projects/sofa/share");
    std::string fileName = sofa::helper::system::DataRepository.getFile(argFile);
    if(!QFile(fileName.c_str()).exists()) {
        QMessageBox msg;
        msg.setText("Cannot find the scene file");
        msg.setInformativeText(QString("While looking for '%1'").arg(argFile.c_str()));
        msg.exec();
        return 1;
    }

    QProgressDialog progress;
    progress.setWindowTitle("Opening SOFA scene");
    progress.setRange(0,2);
    progress.setMinimumDuration(0);
    progress.show();
    progress.setLabelText("Initializing SOFA...");
    QApplication::processEvents();

    // Initialize sofa
    sofa::core::ExecParams::defaultInstance()->setAspectID(0);
    sofa::simulation::common::init();
    sofa::simulation::tree::init();
    sofa::simulation::graph::init();
    sofa::component::initComponentBase();
    sofa::component::initComponentCommon();
    sofa::component::initComponentGeneral();
    sofa::component::initComponentAdvanced();
    sofa::component::initComponentMisc();

    // Set search path for plugins to the binary dir and one above it
    std::vector<std::string> &sp = sofa::helper::system::PluginManager::getInstance().getSearchPaths();
    std::string appPath = QApplication::applicationDirPath().toStdString();
    sp.push_back(appPath);
    sp.push_back(appPath + "/../lib");
    sofa::simulation::setSimulation(new sofa::simulation::graph::DAGSimulation());

    progress.setLabelText("Loading scene file ...");
    QApplication::processEvents();
    Node::SPtr root = sofa::simulation::getSimulation()->load(fileName.c_str());
    progress.setValue(1);

    progress.setLabelText("Initializing scene ...");
    QApplication::processEvents();
    sofa::simulation::getSimulation()->init(root.get());
    progress.setValue(2);
    QApplication::processEvents();


    Visualizer *w = new Visualizer(root);
    w->setWindowFilePath(fileName.c_str());
    w->setMessage(fileName.c_str());
    w->show();
    QApplication::setActiveWindow(w);

    int ret = QApplication::exec();

    sofa::simulation::graph::cleanup();
    sofa::simulation::tree::cleanup();
    sofa::simulation::common::cleanup();

    return ret;
}

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  app.setApplicationName("SurfLab TIPS");
  app.setApplicationDisplayName("SurfLab TIPS");
  app.setApplicationVersion("0.1");
  app.setWindowIcon(QIcon(":/icons/application-icon.png"));

  if(argc < 2) {
      QString fileName = QFileDialog::getOpenFileName(NULL, "Open SOFA scene", QString(), "Scene files (*.scn);;SaLua files (*.salua);;XML files (*.xml);; All files (*.*)");
      if(!fileName.isEmpty())
          return openWindow(fileName.toStdString());
      else
          return 0;
  } else
    return openWindow(argv[1]);
}

