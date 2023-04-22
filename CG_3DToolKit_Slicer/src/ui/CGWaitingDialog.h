#ifndef CGWAITINGDIALOG_H
#define CGWAITINGDIALOG_H

#include <QObject>
#include <QDialog>

class QLabel;
class QProgressBar;
class CGWaitingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGWaitingDialog(QWidget *parent = nullptr);

public:
    void InitUi();

public:
    QLabel *m_pWaitingMsgLb;
    QProgressBar *m_pProgressBar;

};

#endif // CGWAITINGDIALOG_H
