#ifndef CGLOCALDATAFILEDIALOG_H
#define CGLOCALDATAFILEDIALOG_H

#include <QObject>
#include <QDialog>

class QLabel;
class QLineEdit;
class NodeBlock;
class CGLocalDataFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CGLocalDataFileDialog(QWidget *parent = nullptr);
    ~CGLocalDataFileDialog() = default;

public:
    void InitUi();
    void InitConnections();

    void SetCurrentNodeBlock(NodeBlock *nodeblock);
    NodeBlock *GetCurrentNodeBlock() const;

private slots:
    void OnLoadImage();
    void OnLoadPointCloud();

public:
    QLineEdit *m_pFilePath;
    QPushButton *m_pFileBtn;
    int m_2d3dfile;

private:
    QLabel *m_pLabel;
    QPushButton *m_pOKBtn;
    QString m_FileName;
    NodeBlock *m_pNodeBlock;
};

#endif // CGLOCALDATAFILEDIALOG_H
