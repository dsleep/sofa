#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/graph/DAGSimulation.h>
#include <sofa/simulation/common/init.h>
#include <sofa/simulation/graph/init.h>
#include <sofa/simulation/tree/init.h>
#include <SofaComponentBase/initComponentBase.h>
#include <SofaComponentCommon/initComponentCommon.h>
#include <SofaComponentGeneral/initComponentGeneral.h>
#include <SofaComponentAdvanced/initComponentAdvanced.h>
#include <SofaComponentMisc/initComponentMisc.h>
#include <sofa/core/visual/DrawToolGL.h>


#include <QGLWidget>
#include <QKeyEvent>
#include <QTimer>

using sofa::simulation::Node;

class Visualizer : public QGLWidget {
  Q_OBJECT


public:
  Visualizer(Node::SPtr sceneRoot);

  QString message() const { return _message; }
  void setMessage(const QString &message) { _message = message; }
  Node *sceneRoot() const { return _sceneRoot.get(); }
  bool animationRunning()const;
  void setAnimationRunning(const bool& v);

public slots:
  void stepAnimation();

protected:
  virtual void resizeGL(int w, int h);
  virtual void paintGL();
  virtual void initializeGL();


  virtual void keyPressEvent(QKeyEvent* e);
  virtual void wheelEvent(QWheelEvent* e);

private:
  Node::SPtr _sceneRoot;
  sofa::core::visual::DrawToolGL _drawTool;
  float _zoom, _aspectRatio, _animationSpeed;
  QTimer _animationTimer;
  QString _message;
};

#endif // VISUALIZER_HPP

