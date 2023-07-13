#include "CGAlgorithmArgumentsDialog.h"
#include <QLabel>
#include <QPushButton>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


CGAlgorithmArgumentsDialog::CGAlgorithmArgumentsDialog(QWidget *parent) : QDialog(parent)
{
    InitUi();
    InitConnections();
}

void CGAlgorithmArgumentsDialog::InitUi()
{
    setWindowTitle(tr(u8"功能算子  参数设置"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(800, 600);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addWidget(NodeBlockWidget::getInstance());
    setLayout(pMainLayout);
}

void CGAlgorithmArgumentsDialog::InitConnections()
{

}
