#include "CGUsersLoginDialog.h"
#include <QLabel>
#include <QRadioButton>
#include <QPushButton>
#include <QLineEdit>
#include <QMessageBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

CGUsersLoginDialog::CGUsersLoginDialog(QWidget *parent): QDialog(parent)
{
    InitUi();
    InitConnections();
}

void CGUsersLoginDialog::InitUi()
{
    setWindowTitle(tr(u8"用户登录"));
    setWindowFlag(Qt::WindowContextHelpButtonHint, false);
    setWindowIcon(QIcon(":/res/icon/slicer.png"));
    resize(400, 280);

    m_pAdmin = new QRadioButton(tr(u8"管理员"), this);
    m_pAdmin->setIcon(QIcon(":/res/icon/users.png"));
    m_pOperator = new QRadioButton(tr(u8"操作员"), this);
    m_pOperator->setIcon(QIcon(":/res/icon/users.png"));
    m_pOperator->setChecked(true);

    m_pPasswordLb = new QLabel(tr(u8"密码："), this);
    m_pPassword = new QLineEdit(this);
    m_pPassword->setFixedHeight(30);
    m_pPassword->setEchoMode(QLineEdit::Password);

    m_pOKBtn = new QPushButton(tr(u8"确定"), this);
    m_pCancelBtn = new QPushButton(tr(u8"取消"), this);

    QGroupBox *pUsersGroupBox = new QGroupBox();
    pUsersGroupBox->setTitle(tr(u8"用户管理"));
    QVBoxLayout *pUsersLayout = new QVBoxLayout(pUsersGroupBox);

    QHBoxLayout *pAdminLayout = new QHBoxLayout();
    pAdminLayout->addWidget(m_pAdmin);
    pAdminLayout->addStretch();

    QHBoxLayout *pOperatorLayout = new QHBoxLayout();
    pOperatorLayout->addWidget(m_pOperator);
    pOperatorLayout->addStretch();

    QHBoxLayout *pPasswordLayout = new QHBoxLayout();
    pPasswordLayout->addStretch();
    pPasswordLayout->addWidget(m_pPasswordLb);
    pPasswordLayout->addWidget(m_pPassword);
    pPasswordLayout->addStretch();

    pUsersLayout->addSpacing(20);
    pUsersLayout->addLayout(pOperatorLayout);
    pUsersLayout->addSpacing(20);
    pUsersLayout->addLayout(pAdminLayout);
    pUsersLayout->addLayout(pPasswordLayout);
    pUsersLayout->addStretch();

    QHBoxLayout *pBottomLayout = new QHBoxLayout();
    pBottomLayout->addStretch();
    pBottomLayout->addWidget(m_pOKBtn);
    pBottomLayout->addWidget(m_pCancelBtn);

    QVBoxLayout *pMainLayout = new QVBoxLayout();
    pMainLayout->addWidget(pUsersGroupBox);
    pMainLayout->addLayout(pBottomLayout);

    setLayout(pMainLayout);
}

void CGUsersLoginDialog::InitConnections()
{
    connect(m_pOKBtn, &QPushButton::clicked, this, &CGUsersLoginDialog::OnOK);
    connect(m_pCancelBtn, &QPushButton::clicked, this, &CGUsersLoginDialog::OnCancel);
}

void CGUsersLoginDialog::OnOK()
{
    if (m_pAdmin->isChecked())
    {
        if (m_pPassword->text().trimmed() == "cg" ||
            m_pPassword->text().trimmed() == "CG" )
        {
            m_User = "Administrator";
        }
        else
        {
            QMessageBox::warning(this, tr(u8"警告"), tr(u8"请输入正确的密码！"));
            return;
        }
    }
    else
    {
        m_User = "Operator";
    }
    m_pPassword->clear();
    emit SignalUsersLogin(m_User);
    this->accept();
    this->close();
}

void CGUsersLoginDialog::OnCancel()
{
    this->reject();
    this->close();
}
