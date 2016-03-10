#include "Visualizer.hpp"
#include <QApplication>

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

  // Loading the scene
  std::string fileName = sofa::helper::system::DataRepository.getFile(argv[1]);
  Node::SPtr root = sofa::simulation::getSimulation()->load(fileName.c_str());
  sofa::simulation::getSimulation()->init(root.get());


  Visualizer *w = new Visualizer(root);
  w->setWindowFilePath(fileName.c_str());
  w->setMessage(fileName.c_str());
  w->show();
  app.setActiveWindow(w);
  return app.exec();
}

