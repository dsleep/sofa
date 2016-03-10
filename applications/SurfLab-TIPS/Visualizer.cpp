#include "Visualizer.hpp"


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

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-30 * _aspectRatio, 30 * _aspectRatio, -30, 30, -30, 30);
    glScalef(_zoom, _zoom, _zoom);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    int margin = 10;
    renderText(margin, height() - margin, _message);
    glColor4f(0.5f, 0.5f, 0.6f, 1.0f);
    sofa::core::visual::VisualParams* vparams = sofa::core::visual::VisualParams::defaultInstance();
    vparams->drawTool() = &_drawTool;
    sofa::simulation::getSimulation()->draw(vparams, _sceneRoot.get());
}

void Visualizer::initializeGL() {
    glewInit();
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);
    sofa::simulation::getSimulation()->initTextures(_sceneRoot.get());
}



void Visualizer::keyPressEvent(QKeyEvent *e){
    if(e->key() == Qt::Key_Escape) close();
    if(e->key() == Qt::Key_Space && e->modifiers() == Qt::ControlModifier) setAnimationRunning(!animationRunning());
    QGLWidget::keyPressEvent(e);
}

void Visualizer::wheelEvent(QWheelEvent *e) {
    _zoom *= exp(e->delta() / 1000.0f);
    _message = QString("Zoom: %1").arg(_zoom);
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
