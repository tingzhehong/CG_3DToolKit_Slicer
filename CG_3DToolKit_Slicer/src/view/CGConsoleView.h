#ifndef CGCONSOLEVIEW_H
#define CGCONSOLEVIEW_H

#include <QWidget>
#include <CGBaseWidget.h>

class QListWidget;
class CGConsoleView : public CGBaseWidget
{
    Q_OBJECT

private:
    explicit CGConsoleView(CGBaseWidget *parent = nullptr);
    ~CGConsoleView() = default;

signals:

private slots:
    void OnConsoleClear();

public:
    static CGConsoleView *getInstance();
    void InitUi() override;
    void InitConnections() override;
    void ConsoleOut(const QString msg);

    QListWidget *m_pListWidget;
    QAction *m_action;

private:
    static CGConsoleView *m_CGConsoleView;

};

#endif // CGCONSOLEVIEW_H
