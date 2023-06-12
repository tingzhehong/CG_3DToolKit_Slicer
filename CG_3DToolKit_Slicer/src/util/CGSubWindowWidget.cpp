#include "CGSubWindowWidget.h"
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QCloseEvent>
#include <QIcon>
#include <QDebug>

CGSubWindowWidget::CGSubWindowWidget()
{
    setGeometry(0, 0, 800, 600);
    setStyleSheet("background-color:rgb(25, 50, 75)");
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
}

void CGSubWindowWidget::closeEvent(QCloseEvent *closeEvent)
{
    closeEvent->ignore();
}
