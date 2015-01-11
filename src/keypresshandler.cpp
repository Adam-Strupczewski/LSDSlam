#include "keypresshandler.h"

#include <QEvent>
#include <QKeyEvent>

KeyPressHandler::KeyPressHandler(QObject *parent) :
    QObject(parent)
{
}

bool KeyPressHandler::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
        qDebug("Handled key press %d", keyEvent->key());
        if (keyEvent->key()==32){
            *start = true;
        }else if (keyEvent->key()==82){
            *reset = true;
        }
        return true;
    } else {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

void KeyPressHandler::setStartVar(bool *b){
    start = b;
}

void KeyPressHandler::setResetVar(bool *b){
    reset = b;
}
