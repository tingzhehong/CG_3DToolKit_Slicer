#ifndef CGABOUTDIALOG_H
#define CGABOUTDIALOG_H

#include <QObject>
#include <QDialog>

class QLabel;
class CGAboutDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGAboutDialog(QWidget *parent = nullptr);

public slots:
    void OnOK();

public:
    void InitUi();
    void InitConnections();

private:
    QLabel *m_pAboutLb;
    QLabel *m_pVersion;
    QLabel *m_pAuthor;
    QPushButton *m_pOKBtn;
};

#endif // CGABOUTDIALOG_H
