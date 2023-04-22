#ifndef CGUSERSLOGIN_H
#define CGUSERSLOGIN_H

#include <QObject>
#include <QDialog>

class QLabel;
class QRadioButton;
class QPushButton;
class QLineEdit;

class CGUsersLoginDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGUsersLoginDialog(QWidget *parent = nullptr);

signals:
    void SignalUsersLogin(const QString user);

public:
    void InitUi();
    void InitConnections();

private:
    void OnOK();
    void OnCancel();

private:
    QRadioButton *m_pAdmin;
    QRadioButton *m_pOperator;

    QLabel *m_pPasswordLb;
    QLineEdit *m_pPassword;

    QPushButton *m_pOKBtn;
    QPushButton *m_pCancelBtn;

    QString m_User;

};

#endif // CGUSERSLOGIN_H
