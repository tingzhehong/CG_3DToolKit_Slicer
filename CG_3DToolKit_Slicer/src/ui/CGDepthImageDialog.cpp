#include "CGDepthImageDialog.h"
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


CGDepthImageDialog::CGDepthImageDialog(QWidget *parent): QDialog(parent)
{
    setWindowTitle(tr(u8"深度图像设置"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(360, 200);
    InitUi();
    InitConnections();
}

void CGDepthImageDialog::InitUi()
{

}

void CGDepthImageDialog::InitConnections()
{

}
