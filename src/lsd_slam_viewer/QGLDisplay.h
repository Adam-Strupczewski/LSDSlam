#include <QGLWidget>
#include <QWidget>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>

#ifndef QGLDISPLAY_H
#define QGLDISPLAY_H

class QGLDisplay : public QGLWidget
{
public:
    QGLDisplay(QWidget* parent = NULL);
    void setImage(const QImage& image);
protected:
    void paintEvent(QPaintEvent*);
private:
    QImage img;
};

QGLDisplay::QGLDisplay(QWidget* parent)
    : QGLWidget(parent)
{
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

#endif // QGLDISPLAY_H
