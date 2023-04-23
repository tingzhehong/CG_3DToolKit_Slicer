#ifndef CGDISORDERDIALOG_H
#define CGDISORDERDIALOG_H

#include <QDialog>

class QLabel;
class QLineEdit;
class QPushButton;

class CGDisOrderDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGDisOrderDialog(QWidget *parent = nullptr);

    void InitUi();
    void InitConnections();

public:
    int m_Width = 0;
    int m_Height = 0;

private:
    QLabel *m_TipLB;
    QLabel *m_WidthLB;
    QLabel *m_HeightLB;

public:
    QLineEdit *m_WidthLE;
    QLineEdit *m_HeightLE;
    QPushButton *m_OKBtn;

};

#endif // CGDISORDERDIALOG_H
