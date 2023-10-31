﻿#ifndef CGSCRIPTCPPEDITOR_H
#define CGSCRIPTCPPEDITOR_H

#include <QObject>
#include <QDialog>

class QLabel;
class QLineEdit;
class QTextEdit;
class QPushButton;
class QComboBox;
class QTableWidget;
class NodeBlock;
class CGScriptCppEditor : public QDialog
{
    Q_OBJECT

public:
    explicit CGScriptCppEditor(QWidget *parent = nullptr);
    ~CGScriptCppEditor() = default;

public:
    void InitUi();
    void InitConnections();

    void SetCurrentNodeBlock(NodeBlock *nodeblock);
    NodeBlock *GetCurrentNodeBlock() const;

private:
    void InitTableWidget();

public:
    QTextEdit *m_pTextEdit;

private:
    NodeBlock *m_pNodeBlock;

    QTableWidget *m_pTableWidget;

    QComboBox *m_Input_1;
    QComboBox *m_Input_2;
    QComboBox *m_Input_3;

    QComboBox *m_Output_1;
    QComboBox *m_Output_2;
    QComboBox *m_Output_3;

    QPushButton *m_pItemSetBtn;
    QPushButton *m_pScriptSetBtn;

};

#endif // CGSCRIPTCPPEDITOR_H
