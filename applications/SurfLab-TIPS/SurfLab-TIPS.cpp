#include "Visualizer.hpp"
#include <QApplication>
#include <QMessageBox>
#include <QFileDialog>
int openWindow(std::string argFile) {
    // Loading the scene
    std::string fileName = sofa::helper::system::DataRepository.getFile(argFile);
    if(!QFile(fileName.c_str()).exists()) {
        QMessageBox msg;
        msg.setText("Cannot find the scene file");
        msg.setInformativeText(QString("While looking for '%1'").arg(argFile.c_str()));
        msg.exec();
        return 1;
    }
    Node::SPtr root = sofa::simulation::getSimulation()->load(fileName.c_str());
    sofa::simulation::getSimulation()->init(root.get());

    Visualizer *w = new Visualizer(root);
    w->setWindowFilePath(fileName.c_str());
    w->setMessage(fileName.c_str());
    w->show();
    QApplication::setActiveWindow(w);
    return  QApplication::exec();
}

const bool useFileDialog = true;

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  app.setApplicationName("SurfLab TIPS");


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
  sofa::helper::system::DataRepository.addFirstPath("/home/saleh/projects/sofa/examples");
  sofa::helper::system::DataRepository.addFirstPath("/home/saleh/projects/sofa/share");
  sofa::simulation::setSimulation(new sofa::simulation::graph::DAGSimulation());

  int ret;
  if(argc < 2) {
      if(useFileDialog){
          QString fileName = QFileDialog::getOpenFileName(NULL, "Open SOFA scene", QString(), "Scene files (*.scn);;SaLua files (*.salua);;XML files (*.xml);; All files (*.*)");
          if(!fileName.isEmpty())
              openWindow(fileName.toStdString());
          else
              return 0;
      } else {
          QMessageBox msg;
          msg.setText("Please provide a scene filename as the first argument.");
          msg.setInformativeText(QString("Usage: %1 <filename>").arg(argv[0]));
          msg.exec();
          return 2;
      }
  } else
    ret = openWindow(argv[1]);

  sofa::simulation::graph::cleanup();
  sofa::simulation::tree::cleanup();
  sofa::simulation::common::cleanup();

  return ret;

}

