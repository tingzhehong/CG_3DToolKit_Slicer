#ifndef CGLOCALDATAFILEDIALOG_H
#define CGLOCALDATAFILEDIALOG_H

#include <QObject>
#include <QDialog>

class QLabel;
class QLineEdit;
class CGLocalDataFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGLocalDataFileDialog(QWidget *parent = nullptr);
    ~CGLocalDataFileDialog() = default;

public:
    void InitUi();
    void InitConnections();

public:
    QPushButton *m_pFileBtn;

private:
    QLabel *m_pLabel;
    QLineEdit *m_pFilePath;
    QPushButton *m_pOKBtn;
};

#endif // CGLOCALDATAFILEDIALOG_H
