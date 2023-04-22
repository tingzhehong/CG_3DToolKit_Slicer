#include "CGProfileForm2D.h"
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsPixmapItem>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QDebug>

CGProfileForm2D::CGProfileForm2D(QWidget *parent) : QWidget(parent)
{
    m_pScene = new QGraphicsScene();
    m_pPixmap = new QPixmap();
    m_pItem = new QGraphicsPixmapItem();
    m_pGraphicsView = new CGGraphicsView(this);

    InitUi();
    InitConnections();
}

void CGProfileForm2D::InitUi()
{
    QGridLayout *pMainLayout = new QGridLayout();
    pMainLayout->addWidget(m_pGraphicsView);

    setLayout(pMainLayout);
    setVisible(true);
}

void CGProfileForm2D::InitConnections()
{

}
