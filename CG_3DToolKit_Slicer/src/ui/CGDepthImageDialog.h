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
    QPushButton *m_OKBtn;
    QPushButton *m_CancelBtn;

};

#endif // CGDEPTHIMAGEDIALOG_H
