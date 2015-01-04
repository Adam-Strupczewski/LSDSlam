#include "QGLDisplay.h"

QGLDisplay::QGLDisplay(QWidget* parent)
    : QGLWidget(parent)
{
    img = QImage("/home/adam/1.png");
}

void QGLDisplay::setImage(const QImage& image)
{
    img = image;
}

void QGLDisplay::paintEvent(QPaintEvent*)
{
    QPainter p(this);

    //Set the painter to use a smooth scaling algorithm.
    //p.SetRenderHint(QPainter::SmoothPixmapTransform, 1);

    p.drawImage(this->rect(), img);
}
