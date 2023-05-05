#ifndef CGNODEVIEW_H
#define CGNODEVIEW_H

#include <CGBaseWidget.h>
#include <NodeView.h>
#include <NodeItem.h>

class CGNodeView : public CGBaseWidget
{
    Q_OBJECT

public:
    explicit CGNodeView(QWidget *parent = nullptr);
    ~CGNodeView();

public:
    void InitUi() override;
    void InitConnections() override;
    void CreateMathsNodeItem(const QString toolname);
    void CreateLogicsNodeItem(const QString toolname);

private:
    void Test();
    void Verify();

public:
    NodeView *m_NodeView;
};

#endif // CGNODEVIEW_H
