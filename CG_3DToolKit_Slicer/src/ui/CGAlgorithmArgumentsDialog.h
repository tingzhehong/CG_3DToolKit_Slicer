#ifndef CGALGORITHMARGUMENTSDIALOG_H
#define CGALGORITHMARGUMENTSDIALOG_H

#include <QObject>
#include <QDialog>
#include <NodeBlockWidget.h>

class QLabel;
class QPushButton;
class CGAlgorithmArgumentsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGAlgorithmArgumentsDialog(QWidget *parent = nullptr);

public:
    void InitUi();
    void InitConnections();

signals:
    void SignalSetArguments();

public slots:
    void OnOK();
    void OnCancel();
    void OnShapeItemValue(const QString msg);

protected:
    void closeEvent(QCloseEvent *closeEvent);

private:
    QLabel *m_StatusLb;
    QPushButton *m_pOKBtn;
    QPushButton *m_pCancelBtn;

    QFont font;
    QPalette pe;
};

#endif // CGALGORITHMARGUMENTSDIALOG_H
