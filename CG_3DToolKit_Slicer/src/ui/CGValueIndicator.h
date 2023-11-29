#ifndef CGVALUEINDICATOR_H
#define CGVALUEINDICATOR_H

#include <QObject>
#include <QDialog>

class QTextEdit;
class QPushButton;
class CGValueIndicator : public QDialog
{
    Q_OBJECT

public:
    explicit CGValueIndicator(QWidget *parent = nullptr);
    ~CGValueIndicator() = default;

public:
    void InitUi();
    void InitConnections();
    void LoadJsonValueText(const QString str);

private:
    QTextEdit *m_pTextEdit;
    QPushButton *m_pCloseBtn;

};

#endif // CGVALUEINDICATOR_H
