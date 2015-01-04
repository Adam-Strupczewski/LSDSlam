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

#endif // QGLDISPLAY_H
