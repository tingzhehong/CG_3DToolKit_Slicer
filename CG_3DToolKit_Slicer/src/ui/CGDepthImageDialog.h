#ifndef CGDEPTHIMAGEDIALOG_H
#define CGDEPTHIMAGEDIALOG_H

#include <QDialog>

class QLabel;
class QLineEdit;
class QPushButton;

class CGDepthImageDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGDepthImageDialog(QWidget *parent = nullptr);

    void InitUi();
    void InitConnections();

public:
    float xPitch = 0;
    float yPitch = 0;
    float upLimit = 99;
    float downLimit = -99;

private:
    QLabel *m_TipLB;
    QLabel *m_xPitchLB;
    QLabel *m_yPitchLB;
    QLabel *m_UpLimitLB;
    QLabel *m_DownLimitLB;

    QLineEdit *m_xPitchLE;
    QLineEdit *m_yPitchLE;
    QLineEdit *m_UpLimitLE;
    QLineEdit *m_DownLimitLE;

    QPushButton *m_OKBtn;
    QPushButton *m_CancelBtn;

};

#endif // CGDEPTHIMAGEDIALOG_H
