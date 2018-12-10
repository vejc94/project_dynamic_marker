#include "openglwindow.h"

#include <QtCore/QCoreApplication>

#include <QtGui/QOpenGLContext>
#include <QtGui/QOpenGLPaintDevice>
#include <QtGui/QPainter>

OpenGLWindow::OpenGLWindow(QWindow *parent)
    : QWindow(parent)
    , m_animating(false)
    , m_context(0)
    , m_device(0)
{
    setSurfaceType(QWindow::OpenGLSurface);
}

OpenGLWindow::~OpenGLWindow()
{
    delete m_device;
}
void OpenGLWindow::render(QPainter *painter)
{
    //To disable unused variable warnings.
    Q_UNUSED(painter);
}

void OpenGLWindow::initialize()
{
}

void OpenGLWindow::render()
{
    if (!m_device)
        m_device = new QOpenGLPaintDevice;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    m_device->setSize(size());

    QPainter painter(m_device);
    render(&painter);
}
// The renderLater() function simply calls QWindow::requestUpdate() to schedule an update for when the system is ready to repaint.
void OpenGLWindow::renderLater()
{
    //	Schedules a QEvent::UpdateRequest event to be delivered to this window.
    requestUpdate();
}

/// Subclasses of QWindow should reimplement event(), intercept the event and call the application's rendering code,
/// then call the base class implementation.
bool OpenGLWindow::event(QEvent *event)
{
    switch (event->type()) {
    case QEvent::UpdateRequest:
        renderNow();
        return true;
    default:
        return QWindow::event(event);
    }
}

void OpenGLWindow::exposeEvent(QExposeEvent *event)
{
    Q_UNUSED(event);

    if (isExposed())
        renderNow();
}

void OpenGLWindow::renderNow()
{
    if (!isExposed())
        return;

    bool needsInitialize = false;

    if (!m_context) {
        m_context = new QOpenGLContext(this);
        m_context->setFormat(requestedFormat());
        m_context->create();

        needsInitialize = true;
    }

    //make the context current
    m_context->makeCurrent(this);

    if (needsInitialize) {
        /// call initialize() for the sake of the sub class
        /// and initializeOpenGLFunctions() in order for the QOpenGLFunctions super class
        /// to be associated with the correct QOpenGLContext
        initializeOpenGLFunctions();
        initialize();
    }
    //call render() to do the actual rendering
    render();

    ///finally we schedule for the rendered contents to be made visible by calling QOpenGLContext::swapBuffers()
    /// with the OpenGLWindow as parameter.
    m_context->swapBuffers(this);

    if (m_animating)
        renderLater();
}

void OpenGLWindow::setAnimating(bool animating)
{
    m_animating = animating;

    if (animating)
        renderLater();
}
