#include "Visualizer.hpp"

#include <SofaBaseVisual/VisualStyle.h>

#include <QAction>
#include <QMatrix4x4>
#include <QMenu>
#include <QProgressDialog>
#include <QApplication>



Visualizer::Visualizer(Node::SPtr sceneRoot) :  _sceneRoot(sceneRoot), _zoom(1.0f), _animationSpeed(1.0f)
{
    connect(&_animationTimer,SIGNAL(timeout()), this, SLOT(stepAnimation()));
    _animationTimer.setInterval(10);

    _visualParameters = sofa::core::visual::VisualParams::defaultInstance();
    _visualParameters->drawTool() = &_drawTool;

    _simulation = sofa::simulation::getSimulation();

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

    sofa::component::visualmodel::VisualStyle* visualStyle = NULL;
    _sceneRoot->get(visualStyle);
    const sofa::core::visual::DisplayFlags &df = visualStyle ? visualStyle->displayFlags.getValue() :  _visualParameters->displayFlags();

    QAction* d1 = displayMenu->addAction("Visual Models");
    connect(d1,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d1->setShortcut(QKeySequence("Ctrl+1"));
    d1->setCheckable(true);
    d1->setChecked(true);
    d1->setData(1);

    QAction* d2 = displayMenu->addAction("Behavioral Models");
    connect(d2,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d2->setShortcut(QKeySequence("Ctrl+2"));
    d2->setCheckable(true);
    d2->setChecked(df.getShowBehaviorModels());
    d2->setData(2);

    QAction* d3 = displayMenu->addAction("Collision Models");
    connect(d3,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d3->setShortcut(QKeySequence("Ctrl+3"));
    d3->setCheckable(true);
    d3->setChecked(df.getShowCollisionModels());
    d3->setData(3);

    QAction* d4 = displayMenu->addAction("Bounding Collision Models");
    connect(d4,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d4->setShortcut(QKeySequence("Ctrl+4"));
    d4->setCheckable(true);
    d4->setChecked(df.getShowBoundingCollisionModels());
    d4->setData(4);

    QAction* d5 = displayMenu->addAction("Mappings");
    connect(d5,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d5->setShortcut(QKeySequence("Ctrl+5"));
    d5->setCheckable(true);
    d5->setChecked(df.getShowMappings());
    d5->setData(5);

    QAction* d6 = displayMenu->addAction("Mechanical Mappings");
    connect(d6,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d6->setShortcut(QKeySequence("Ctrl+6"));
    d6->setCheckable(true);
    d6->setChecked(df.getShowMechanicalMappings());
    d6->setData(6);

    QAction* d7 = displayMenu->addAction("Force Fields");
    connect(d7,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d7->setShortcut(QKeySequence("Ctrl+7"));
    d7->setCheckable(true);
    d7->setChecked(df.getShowForceFields());
    d7->setData(7);

    QAction* d8 = displayMenu->addAction("Interaction Force Fields");
    connect(d8,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d8->setShortcut(QKeySequence("Ctrl+8"));
    d8->setCheckable(true);
    d8->setChecked(df.getShowInteractionForceFields());
    d8->setData(8);

    QAction* d9 = displayMenu->addAction("Wireframe");
    connect(d9,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d9->setShortcut(QKeySequence("Ctrl+9"));
    d9->setCheckable(true);
    d9->setChecked(df.getShowWireFrame());
    d9->setData(9);

    QAction* d0 = displayMenu->addAction("Normals");
    connect(d0,SIGNAL(toggled(bool)),this, SLOT(toggleDisplayFlag(bool)));
    d0->setShortcut(QKeySequence("Ctrl+0"));
    d0->setCheckable(true);
    d0->setChecked(df.getShowNormals());
    d0->setData(0);

    addAction(display);

    QAction *reset = new QAction("Reset scene", this);
    reset->setShortcut(QKeySequence("Ctrl+L"));
    connect(reset, SIGNAL(triggered()), this, SLOT(resetScene()));
    addAction(reset);

    QAction *reload = new QAction("Reload scene from disk", this);
    reload->setShortcut(QKeySequence("Ctrl+R"));
    connect(reload, SIGNAL(triggered()), this, SLOT(reloadScene()));
    addAction(reload);

    QAction *close = new QAction("Close", this);
    close->setShortcut(QKeySequence("Escape"));
    connect(close,SIGNAL(triggered()),this,SLOT(close()));
    addAction(close);

    setContextMenuPolicy(Qt::ActionsContextMenu);
}


/*!
 * \brief Visualizer::reloadScene
 * Reload the scene file from disk
 *
 * We rely on the fact that the scene always has a valid file path.
 */
void Visualizer::reloadScene(){
    hide();
    setAnimationRunning(false);
    QString fileName = windowFilePath();
    QProgressDialog progress(this);
    progress.show();
    progress.setRange(0, 3);
    progress.setWindowTitle("Reloading the scene");
    progress.setLabelText("Unloading current scene...");
    QApplication::processEvents();
    _simulation->unload(_sceneRoot);
    progress.setValue(1);
    progress.setLabelText("Loading the scene from disk...");
    QApplication::processEvents();
    _sceneRoot = _simulation->load(fileName.toUtf8().data());
    assert(_sceneRoot != NULL);
    progress.setValue(2);
    progress.setLabelText("Initializing the scene...");
    QApplication::processEvents();
    _simulation->init(_sceneRoot.get());
    progress.setValue(3);
    progress.setLabelText("Loading done");
    QApplication::processEvents();
    setMessage("Reload from disk finished");
    QApplication::processEvents();
    show();
    updateGL();
}

void Visualizer::resetScene(){
    _simulation->reset(_sceneRoot.get());
}

void Visualizer::toggleDisplayFlag(bool b){
    QAction *sender = qobject_cast<QAction*>(QObject::sender());
    if(!sender) return;
    if(sender->data().type() != QVariant::Int) return;

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
    case 4:
        df.setShowBoundingCollisionModels(b);
        break;
    case 5:
        df.setShowMappings(b);
        break;
    case 6:
        df.setShowMechanicalMappings(b);
        break;
    case 7:
        df.setShowForceFields(b);
        break;
    case 8:
        df.setShowInteractionForceFields(b);
        break;
    case 9:
        df.setShowWireFrame(b);
        break;
    case 0:
        df.setShowNormals(b);
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
    _simulation->animate(_sceneRoot.get(), _animationSpeed * _animationTimer.interval() / 1000.0f);
    _simulation->updateVisual(_sceneRoot.get());
    updateGL();
}

void Visualizer::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    _aspectRatio = w / float(h);
}

void Visualizer::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

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

    _simulation->draw(_visualParameters, _sceneRoot.get());

    // Write the message text in the corner of the window
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int margin = 10;
    renderText(margin, height() - margin, _message);
}

void Visualizer::initializeGL() {
    glewInit();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    GLfloat lightPosition[4] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    glEnable(GL_DEPTH_TEST);
    _simulation->initTextures(_sceneRoot.get());
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
