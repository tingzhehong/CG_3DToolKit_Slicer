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

private:

};

#endif // CGALGORITHMARGUMENTSDIALOG_H
