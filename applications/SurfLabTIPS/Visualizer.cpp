#include "Visualizer.hpp"

#include <SofaBaseVisual/VisualStyle.h>

#include <QAction>
#include <QMatrix4x4>
#include <QMenu>
#include <QSignalMapper>
static QGLFormat openglFormat(){
    QGLFormat format;
    format.setVersion(3, 2);
    format.setDoubleBuffer(true);
    format.setSamples(8);
    return format;
}


Visualizer::Visualizer(Node::SPtr sceneRoot) : QGLWidget(openglFormat()), _sceneRoot(sceneRoot), _zoom(1.0f), _animationSpeed(1.0f)
{
    connect(&_animationTimer,SIGNAL(timeout()), this, SLOT(stepAnimation()));
    _animationTimer.setInterval(10);

    _visualParameters = sofa::core::visual::VisualParams::defaultInstance();
    _visualParameters->drawTool() = &_drawTool;

    createContextMenu();
}

void Visualizer::createContextMenu() {
    QAction *animate = new QAction("Animate", this);
    animate->setShortcut(QKeySequence("Ctrl+Space"));
    animate->setCheckable(true);
    connect(animate,SIGNAL(toggled(bool)), this, SLOT(setAnimationRunning(bool)));
    addAction(animate);

    QAction *step = new QAction("Step", this);
    step->setShortcut(QKeySequence("Ctrl+N"));
    connect(step,SIGNAL(triggered()),this,SLOT(stepAnimation()));
    addAction(step);

    QAction *fullscreen = new QAction("Fullscreen", this);
    fullscreen->setShortcut(QKeySequence("F11"));
    fullscreen->setCheckable(true);
    connect(fullscreen,SIGNAL(toggled(bool)),this,SLOT(setFullscreen(bool)));
    addAction(fullscreen);

    QAction *display = new QAction("Display", this);
    QMenu *displayMenu = new QMenu(this);
    display->setMenu(displayMenu);

    QAction* d1 = displayMenu->addAction("Visual Models");
    connect(d1,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d1->setShortcut(QKeySequence("Ctrl+1"));
    d1->setCheckable(true);
    d1->setChecked(_visualParameters->displayFlags().getShowVisualModels());
    d1->setData(1);

    QAction* d2 = displayMenu->addAction("Behavioral Models");
    connect(d2,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d2->setShortcut(QKeySequence("Ctrl+2"));
    d2->setCheckable(true);
    d2->setChecked(_visualParameters->displayFlags().getShowBehaviorModels());
    d2->setData(2);

    QAction* d3 = displayMenu->addAction("Collision Models");
    connect(d3,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d3->setShortcut(QKeySequence("Ctrl+3"));
    d3->setCheckable(true);
    d3->setChecked(_visualParameters->displayFlags().getShowCollisionModels());
    d3->setData(3);


    addAction(display);

    QAction *close = new QAction("Close", this);
    close->setShortcut(QKeySequence("Escape"));
    connect(close,SIGNAL(triggered()),this,SLOT(close()));
    addAction(close);

    setContextMenuPolicy(Qt::ActionsContextMenu);
}

void Visualizer::toggleDisplayFlag(bool b){
    QAction *sender = qobject_cast<QAction*>(QObject::sender());
    if(!sender) return;

    sofa::component::visualmodel::VisualStyle* visualStyle = NULL;
    _sceneRoot->get(visualStyle);

    sofa::core::visual::DisplayFlags &df = visualStyle ? *visualStyle->displayFlags.beginEdit() :  _visualParameters->displayFlags();
    setMessage(QString("Set %1 %2").arg(sender->text()).arg(b ? "visible" : "hidden"));
    switch(sender->data().toInt()){
    case 1:
        df.setShowVisualModels(b);
        break;
    case 2:
        df.setShowBehaviorModels(b);
        break;
    case 3:
        df.setShowCollisionModels(b);
        break;
    case 7:
        df.setShowForceFields(b);
        break;
    }
    if(visualStyle) visualStyle->displayFlags.endEdit();
    updateGL();
}

void Visualizer::setFullscreen(bool f){
    if(f)
        showFullScreen();
    else
        showNormal();
}

void Visualizer::stepAnimation() {
    sofa::simulation::getSimulation()->animate(_sceneRoot.get(), _animationSpeed * _animationTimer.interval() / 1000.0f);
    sofa::simulation::getSimulation()->updateVisual(_sceneRoot.get());
    updateGL();
}

void Visualizer::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    _aspectRatio = w / float(h);
}

void Visualizer::paintGL() {
    GLfloat lightPosition[4] = { 1.0, 1.0, 1.0, 1.0 };
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    QMatrix4x4 proj;
    proj.perspective(30, _aspectRatio, 1, 100);
    proj.translate(0, 0, -30);
    proj.scale(_zoom);
    proj.rotate(_sceneRotation);
    proj.translate(_sceneTranslation);

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(proj.data());

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor4f(0.5f, 0.5f, 0.6f, 1.0f);
    sofa::simulation::getSimulation()->draw(_visualParameters, _sceneRoot.get());
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int margin = 10;
    renderText(margin, height() - margin, _message);
}

void Visualizer::initializeGL() {
    glewInit();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    sofa::simulation::getSimulation()->initTextures(_sceneRoot.get());
}


QVector3D Visualizer::mapToHemiSphere(int x, int y) {
    QVector3D p;
    int w = width(), h = height(), l = std::min(w, h);
    p[0] = (x - w/2.0f)/l, p[1] = -(y - h/2.0f)/l;
    float d = p[0]*p[0] + p[1]*p[1];
    p[2] = d < 1.0f ? sqrt(1.0f - d) : 0.0;
    return p;
}


void Visualizer::keyPressEvent(QKeyEvent *e){
    QGLWidget::keyPressEvent(e);
}

void Visualizer::wheelEvent(QWheelEvent *e) {
    _zoom *= exp(e->delta() / 1000.0f);
    _message = QString("Zoom: %1").arg(_zoom);
    updateGL();
}

void Visualizer::mousePressEvent(QMouseEvent *e){
    _lastMousePos = mapToHemiSphere(e->x(), e->y());
}

void Visualizer::mouseMoveEvent(QMouseEvent *e){
    QVector3D pos =  mapToHemiSphere(e->x(),e->y());
    QVector3D diff2D = pos - _lastMousePos; diff2D[2] = 0.0f;
    const float multiplier = 200.0f;

    switch(e->modifiers()){
    case Qt::ShiftModifier:
        _sceneTranslation += _sceneRotation.conjugate().rotatedVector(diff2D) / _zoom * 10;
        setMessage(QString("Translation: (%1,%2,%3)").arg(_sceneTranslation[0]).arg(_sceneTranslation[1]).arg(_sceneTranslation[2]));
        break;
    case Qt::NoModifier: // Rotate
        _sceneRotation =  QQuaternion::fromAxisAndAngle(QVector3D::crossProduct(_lastMousePos,pos),(pos - _lastMousePos).length()*multiplier) * _sceneRotation;
        break;
    }

    _lastMousePos = pos;
    updateGL();
}


bool Visualizer::animationRunning() const { return _animationTimer.isActive(); }

void Visualizer::setAnimationRunning(const bool &v) {
    _message = QString("Animation: %1").arg(v ? "running" : "stopped");
    if(v) {
        _animationTimer.start();
    } else {
        _animationTimer.stop();
    }
    updateGL();
}
