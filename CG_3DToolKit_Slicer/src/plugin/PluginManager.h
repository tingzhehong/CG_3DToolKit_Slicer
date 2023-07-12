#ifndef PLUGINMANAGER_H
#define PLUGINMANAGER_H

#include <QObject>
#include <QList>
#include <QDir>
#include <AlgorithmInterface.h>

class PluginManager : public QObject
{
    Q_OBJECT

public:
    explicit PluginManager(QObject *parent = nullptr);

public:
    bool LoadPlugin(QString path);

    QList<AlgorithmInterface *> PluginList2D() const;
    QList<AlgorithmInterface *> PluginList3D() const;
    QList<AlgorithmInterface *> m_Plugins2D;
    QList<AlgorithmInterface *> m_Plugins3D;
    QList<CG_NODEBLOCK *> m_PluginObjects2D;
    QList<CG_NODEBLOCK *> m_PluginObjects3D;
    QList<int> m_PluginIDs2D;
    QList<int> m_PluginIDs3D;
    QStringList m_PluginNames2D;
    QStringList m_PluginNames3D;
    QStringList m_PluginVersions2D;
    QStringList m_PluginVersions3D;
};

#endif // PLUGINMANAGER_H
